"""Parte ROS del gateway: un nodo rclpy que corre en su propio hilo.

Responsabilidades:
  - Mantener el último estado del robot (pose en el mapa, mapa, scan, Nav2).
  - Publicar la teleoperación validando límites (sin marcha atrás).
  - Watchdog: si la teleoperación deja de llegar, para el robot. Corre en el
    hilo de ROS con reloj de pared, independiente de la parte web.
  - Enviar, cancelar y borrar goals de Nav2. El goal forma parte del estado
    compartido: todos los clientes lo ven hasta que se alcanza o se borra.
  - Emitir eventos (avisos y emergencias) para la barra de avisos.

La parte web (main.py) solo lee estados con los métodos get_* y llama a los
métodos de comando. Todo el acceso compartido va protegido con un lock.
"""
import array
import base64
import collections
import itertools
import logging
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

log = logging.getLogger('gateway.ros')


class CommandRejected(ValueError):
    """Orden rechazada (fuera de límites, Nav2 no disponible, ...)."""


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}

# Estados de navegación en los que hay una ruta en ejecución
NAV_RUNNING = ('sending', 'active')


class RosBridge(Node):

    def __init__(self, cfg):
        super().__init__('dashboard_gateway', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, cfg.USE_SIM_TIME)])
        self.cfg = cfg
        self._lock = threading.Lock()

        # Estado compartido con la parte web
        self._pose = None            # {'x','y','yaw'}
        self._pose_time = 0.0        # time.monotonic() de la última pose
        self._map = None             # dict listo para enviar
        self._map_version = 0
        self._scan = None            # dict listo para enviar
        self._nav = {'state': 'idle', 'goal': None}
        self._ros_ok = False

        # Eventos para la barra de avisos (los últimos 100)
        self._events = collections.deque(maxlen=100)
        self._event_ids = itertools.count(1)

        # Teleoperación / watchdog
        self._last_teleop = 0.0
        self._teleop_active = False
        self._goal_handle = None

        # TF para la pose en el mapa
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(TwistStamped, cfg.TELEOP_TOPIC, 10)

        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, cfg.MAP_TOPIC, self._on_map, map_qos)
        self.create_subscription(LaserScan, cfg.SCAN_TOPIC, self._on_scan,
                                 qos_profile_sensor_data)

        self.nav_client = ActionClient(self, NavigateToPose, cfg.NAV_ACTION)

        self.create_timer(1.0 / cfg.STATE_RATE, self._update_pose)

        # Watchdog y vigilancia de ROS con reloj de pared (STEADY): no
        # dependen del reloj de la simulación ni de la parte web.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self._watchdog, clock=steady)
        self.create_timer(0.5, self._check_ros, clock=steady)

        log.info('Nodo ROS listo. Teleop -> %s, límites vx=[%.2f, %.2f] vy=±%.2f '
                 'wz=±%.2f, watchdog=%.2fs', cfg.TELEOP_TOPIC, cfg.MIN_VX,
                 cfg.MAX_VX, cfg.MAX_VY, cfg.MAX_WZ, cfg.WATCHDOG_TIMEOUT)

    # ---------------------------------------------------------------- eventos
    def _emit(self, level, code, message):
        """level: 'info' | 'warning' | 'emergency'."""
        event = {'type': 'event', 'id': next(self._event_ids), 'level': level,
                 'code': code, 'message': message, 'time': time.time()}
        with self._lock:
            self._events.append(event)

    def emit(self, level, code, message):
        """Evento público para la barra de avisos de todos los clientes."""
        self._emit(level, code, message)

    def get_events_since(self, last_id):
        with self._lock:
            return [e for e in self._events if e['id'] > last_id]

    def last_event_id(self):
        with self._lock:
            return self._events[-1]['id'] if self._events else 0

    # ------------------------------------------------------------------ estado
    def _update_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.cfg.MAP_FRAME, self.cfg.BASE_FRAME, Time())
        except TransformException:
            return
        t = tf.transform.translation
        pose = {'x': round(t.x, 3), 'y': round(t.y, 3),
                'yaw': round(yaw_from_quat(tf.transform.rotation), 4)}
        with self._lock:
            self._pose = pose
            self._pose_time = time.monotonic()

    def _check_ros(self):
        """ROS se considera vivo si la pose del robot sigue llegando."""
        with self._lock:
            age = time.monotonic() - self._pose_time
        ok = self._pose_time > 0 and age < self.cfg.ROS_STALE_TIMEOUT
        if ok != self._ros_ok:
            self._ros_ok = ok
            if ok:
                log.info('Conexión con ROS establecida (pose del robot recibida)')
                self._emit('info', 'ros_ok', 'Conexión con ROS establecida')
            else:
                log.warning('Conexión con ROS PERDIDA: %.1f s sin pose del robot', age)
                self._emit('emergency', 'ros_lost',
                           'Conexión con ROS perdida: sin pose del robot')

    def _on_map(self, msg):
        info = msg.info
        raw = array.array('b', msg.data).tobytes()   # int8: -1 desconocido, 0..100
        data = {
            'type': 'map',
            'width': info.width,
            'height': info.height,
            'resolution': info.resolution,
            'origin': {'x': info.origin.position.x,
                       'y': info.origin.position.y,
                       'yaw': yaw_from_quat(info.origin.orientation)},
            'data': base64.b64encode(raw).decode('ascii'),
        }
        with self._lock:
            self._map = data
            self._map_version += 1

    def _on_scan(self, msg):
        step = max(1, self.cfg.SCAN_DOWNSAMPLE)
        ranges = [round(r, 3) if math.isfinite(r) else None
                  for r in msg.ranges[::step]]
        data = {
            'type': 'scan',
            'frame': msg.header.frame_id,
            'angle_min': msg.angle_min,
            'angle_increment': msg.angle_increment * step,
            'ranges': ranges,
        }
        with self._lock:
            self._scan = data

    def get_state(self):
        with self._lock:
            pose = dict(self._pose) if self._pose else None
            nav = dict(self._nav)
        return {'type': 'state',
                'ros_ok': self._ros_ok,
                'pose': pose,
                'nav': nav,
                'route_running': nav['state'] in NAV_RUNNING,
                'teleop_active': self._teleop_active,
                'nav2_ready': self.nav_client.server_is_ready()}

    def get_config(self):
        """Límites que el frontend necesita para dibujar los controles."""
        c = self.cfg
        return {'type': 'config',
                'limits': {'vx_min': c.MIN_VX, 'vx_max': c.MAX_VX,
                           'vy_max': c.MAX_VY, 'wz_max': c.MAX_WZ},
                'base_frame': c.BASE_FRAME,
                'watchdog_timeout': c.WATCHDOG_TIMEOUT}

    def get_map(self):
        with self._lock:
            return self._map_version, self._map

    def get_scan(self):
        with self._lock:
            return self._scan

    # ---------------------------------------------------------- teleoperación
    def _reject(self, who, text):
        # Se avisa solo a quien envió la orden (main.py le manda un 'error');
        # al resto de clientes no les afecta.
        log.warning('RECHAZADO de %s: %s', who, text)
        raise CommandRejected(text)

    def teleop(self, vx, vy, wz, who='?'):
        c = self.cfg
        for name, value in (('vx', vx), ('vy', vy), ('wz', wz)):
            if not math.isfinite(value):
                self._reject(who, f'{name} no es un número válido')
        if vx < c.MIN_VX - 1e-6:
            self._reject(who, f'vx={vx:.3f}: marcha atrás no permitida')
        if vx > c.MAX_VX:
            self._reject(who, f'vx={vx:.3f} fuera de rango (máx {c.MAX_VX:.3f})')
        if abs(vy) > c.MAX_VY:
            self._reject(who, f'vy={vy:.3f} fuera de rango (máx ±{c.MAX_VY:.3f})')
        if abs(wz) > c.MAX_WZ:
            self._reject(who, f'wz={wz:.3f} fuera de rango (máx ±{c.MAX_WZ:.3f})')

        moving = any(abs(v) > 1e-4 for v in (vx, vy, wz))
        if moving and self._goal_handle is not None:
            log.info('Teleop de %s: se cancela la navegación en curso', who)
            self._emit('warning', 'nav_preempted',
                       'Teleoperación: se cancela la ruta en curso')
            self.cancel_goal(who)

        self._publish_twist(vx, vy, wz)
        self._last_teleop = time.monotonic()
        if moving and not self._teleop_active:
            log.info('Teleop INICIO (%s)', who)
        elif not moving and self._teleop_active:
            log.info('Teleop FIN (%s)', who)
        self._teleop_active = moving

    def stop(self, who='?'):
        log.warning('PARADA solicitada por %s', who)
        self._emit('emergency', 'stop', f'PARADA solicitada ({who})')
        self.cancel_goal(who)
        self._publish_twist(0.0, 0.0, 0.0)
        self._teleop_active = False

    def halt_teleop(self, who='?'):
        """Para la teleoperación en curso sin tocar la navegación (p. ej. al
        cambiar el control de operador)."""
        if self._teleop_active:
            log.info('Teleop detenida (%s)', who)
        self._publish_twist(0.0, 0.0, 0.0)
        self._teleop_active = False

    def _publish_twist(self, vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg.BASE_FRAME
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _watchdog(self):
        if not self._teleop_active:
            return
        silence = time.monotonic() - self._last_teleop
        if silence > self.cfg.WATCHDOG_TIMEOUT:
            self._publish_twist(0.0, 0.0, 0.0)
            self._teleop_active = False
            log.warning('WATCHDOG: %.2f s sin órdenes de teleop -> parada', silence)
            self._emit('warning', 'watchdog',
                       f'Watchdog: teleoperación interrumpida ({silence:.2f} s sin órdenes)')

    # ------------------------------------------------------------------- Nav2
    def send_goal(self, x, y, yaw, who='?'):
        for name, value in (('x', x), ('y', y), ('yaw', yaw)):
            if not math.isfinite(value):
                self._reject(who, f'{name} no es un número válido')
        if not self.nav_client.server_is_ready():
            self._reject(who, 'Nav2 no está disponible (navigate_to_pose sin servidor)')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.cfg.MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        log.info('GOAL de %s: x=%.2f y=%.2f yaw=%.2f', who, x, y, yaw)
        with self._lock:
            self._nav = {'state': 'sending',
                         'goal': {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4)}}
        future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            log.warning('Nav2 rechazó el goal')
            self._emit('warning', 'nav_rejected', 'Nav2 rechazó el goal')
            with self._lock:
                self._nav = {**self._nav, 'state': 'rejected'}
            return
        self._goal_handle = handle
        with self._lock:
            self._nav = {**self._nav, 'state': 'active'}
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        fb = msg.feedback
        with self._lock:
            if self._nav['state'] in NAV_RUNNING:
                self._nav = {**self._nav, 'state': 'active',
                             'distance_remaining': round(fb.distance_remaining, 2)}

    def _on_result(self, future):
        res = future.result()
        status = res.status
        name = GOAL_STATUS_NAMES.get(status, f'status_{status}')
        error_code = int(getattr(res.result, 'error_code', 0) or 0)
        error_msg = str(getattr(res.result, 'error_msg', '') or '')
        self._goal_handle = None

        with self._lock:
            nav = {k: v for k, v in self._nav.items() if k != 'distance_remaining'}
            nav['state'] = name
            if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
                # Goal alcanzado o cancelado: el marcador desaparece
                nav['goal'] = None
            if error_code:
                nav['error_code'] = error_code
            if error_msg:
                nav['error_msg'] = error_msg
            self._nav = nav

        if status == GoalStatus.STATUS_SUCCEEDED:
            log.info('Navegación terminada: %s', name)
            self._emit('info', 'nav_succeeded', 'Goal alcanzado')
        elif status == GoalStatus.STATUS_CANCELED:
            log.info('Navegación terminada: %s', name)
            self._emit('info', 'nav_canceled', 'Navegación cancelada')
        else:
            log.warning('Navegación terminada: %s (error_code=%d %s)',
                        name, error_code, error_msg)
            extra = f' — {error_msg}' if error_msg else (
                f' (código {error_code})' if error_code else '')
            self._emit('warning', 'nav_failed',
                       f'Navegación fallida: {name}{extra}. El goal sigue marcado.')

    def cancel_goal(self, who='?'):
        """Cancela la ruta en curso o, si no hay ninguna, borra el goal marcado."""
        handle = self._goal_handle
        if handle is not None:
            log.info('Cancelación de navegación por %s', who)
            handle.cancel_goal_async()
            return
        with self._lock:
            if self._nav.get('goal') is not None:
                log.info('Goal borrado por %s', who)
                self._nav = {'state': 'idle', 'goal': None}


def start_ros(cfg):
    """Arranca rclpy y el nodo en un hilo propio. Devuelve (nodo, executor, hilo)."""
    rclpy.init()
    node = RosBridge(cfg)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, name='ros-executor', daemon=True)
    thread.start()
    return node, executor, thread


def stop_ros(node, executor):
    try:
        node.stop(who='apagado del gateway')
    except Exception:  # noqa: BLE001 - en el apagado no debe fallar nada
        pass
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
