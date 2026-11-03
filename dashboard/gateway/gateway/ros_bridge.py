"""Parte ROS del gateway: un nodo rclpy que corre en su propio hilo.

Responsabilidades:
  - Mantener el último estado del robot (pose en el mapa, mapa, scan, Nav2).
  - Publicar la teleoperación validando límites (sin marcha atrás).
  - Watchdog: si la teleoperación deja de llegar, para el robot. Corre en el
    hilo de ROS con reloj de pared, independiente de la parte web.
  - Enviar, cancelar y borrar goals de Nav2. El goal forma parte del estado
    compartido: todos los clientes lo ven hasta que se alcanza o se borra.
  - Ejecutar rutas guardadas con un secuenciador propio: un NavigateToPose por
    parada, espera configurable en cada una y política ante fallos
    (cfg.ROUTE_ON_FAILURE).
  - Puntos de trabajo (6.3/6.4): al llegar con Nav2 se lanza el acercamiento
    fino a la pared (acción ApproachWall del nodo kuka_nav/wall_approach.py).
    Vale para «Ir» a un punto de trabajo, para «Ajustar» (solo el acercamiento)
    y para las paradas de trabajo de una ruta. El acercamiento cuenta como
    navegación: mismos bloqueos con el brazo y la teleop, y la PARADA lo cancela.
  - Brazo frente a la pared (5.3): tras un ajuste correcto, la pared del punto
    se añade a la escena de colisiones de MoveIt y se puede llevar el TCP
    delante de cada tornillo de la rejilla («ir a tornillo»). Al iniciar otra
    navegación, la pared se retira.
  - Emitir eventos (avisos y emergencias) para la barra de avisos.
  - Cámara del brazo: guarda la última imagen y la comprime a JPEG bajo
    demanda (una vez por fotograma aunque la vean varios clientes).
  - Brazo: lo gestiona ArmController (arm.py), creado aquí como self.arm.
  - Bloqueo base/brazo: la base no se mueve mientras el brazo se mueve, y al
    revés (lo comprueba arm.py con nav_running()).

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
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

log = logging.getLogger('gateway.ros')

# OpenCV y numpy solo hacen falta para la cámara: si no están, el resto del
# gateway funciona igual y la cámara se marca como no disponible.
try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover
    cv2 = np = None


# Acción propia del acercamiento fino (paquete kuka_interfaces). Si el gateway
# se arranca sin ese paquete en el entorno, todo lo demás funciona y los puntos
# de trabajo se rechazan con un mensaje claro.
try:
    from kuka_interfaces.action import ApproachWall
except ImportError:  # pragma: no cover
    ApproachWall = None

from . import planner as plannerlib
from . import work as workgeo
from .arm import ArmController
from .errors import CommandRejected  # noqa: F401  (main.py la importa desde aquí)


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
        self._lock = threading.RLock()   # _emit puede llamarse con el lock cogido

        # Estado compartido con la parte web
        self._pose = None            # {'x','y','yaw'}
        self._pose_time = 0.0        # time.monotonic() de la última pose
        self._map = None             # dict listo para enviar
        self._map_version = 0
        self._scan = None            # dict listo para enviar
        self._nav = {'mode': None, 'state': 'idle', 'goal': None}
        self._ros_ok = False

        # Eventos para la barra de avisos (los últimos 100)
        self._events = collections.deque(maxlen=100)
        self._event_ids = itertools.count(1)

        # Teleoperación / watchdog
        self._last_teleop = 0.0
        self._teleop_active = False
        self._goal_handle = None
        self._wait_timer = None
        self._job = 0                # trabajo de navegación vigente
        self._work_here = None       # punto de trabajo frente al que está ajustado el robot

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
        if ApproachWall is not None:
            self.approach_client = ActionClient(self, ApproachWall, cfg.APPROACH_ACTION)
        else:
            self.approach_client = None
            log.warning('kuka_interfaces no disponible: los puntos de trabajo no podrán '
                        'ajustarse (source ~/ros2_ws/install/setup.bash antes del gateway)')

        # Cámara del brazo
        self._cam_lock = threading.Lock()
        self._cam_encode_lock = threading.Lock()
        self._cam_msg = None          # última sensor_msgs/Image
        self._cam_seq = 0             # número de fotograma recibido
        self._cam_time = 0.0          # time.monotonic() del último
        self._cam_times = collections.deque(maxlen=30)
        self._cam_jpeg = None         # (seq, bytes) último JPEG generado
        self._cam_error_logged = False
        if cv2 is None:
            log.warning('OpenCV no disponible: la cámara del brazo no se mostrará '
                        '(sudo apt install python3-opencv)')
        self.create_subscription(Image, cfg.CAMERA_TOPIC, self._on_camera,
                                 qos_profile_sensor_data)

        self.create_timer(1.0 / cfg.STATE_RATE, self._update_pose)

        # Watchdog y vigilancia de ROS con reloj de pared (STEADY): no
        # dependen del reloj de la simulación ni de la parte web.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self._watchdog, clock=steady)
        self.create_timer(0.5, self._check_ros, clock=steady)

        self.arm = ArmController(self, cfg)

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

    # ----------------------------------------------------------------- cámara
    def _on_camera(self, msg):
        now = time.monotonic()
        with self._cam_lock:
            self._cam_msg = msg
            self._cam_seq += 1
            self._cam_time = now
            self._cam_times.append(now)

    def camera_status(self):
        with self._cam_lock:
            age = time.monotonic() - self._cam_time if self._cam_time else None
            times = list(self._cam_times)
            msg = self._cam_msg
        fps = 0.0
        if len(times) > 1 and times[-1] > times[0]:
            fps = (len(times) - 1) / (times[-1] - times[0])
        ok = age is not None and age < self.cfg.CAMERA_STALE_TIMEOUT and cv2 is not None
        return {'ok': ok, 'available': cv2 is not None,
                'fps': round(fps, 1) if ok else 0.0,
                'size': [msg.width, msg.height] if msg is not None else None}

    def get_camera_jpeg(self):
        """(seq, jpeg) del último fotograma, comprimiendo solo si es nuevo.
        Devuelve (seq, None) si no hay imagen o no se puede comprimir."""
        if cv2 is None:
            return 0, None
        with self._cam_lock:
            seq, msg = self._cam_seq, self._cam_msg
            cached = self._cam_jpeg
        if msg is None:
            return 0, None
        if cached is not None and cached[0] == seq:
            return cached
        with self._cam_encode_lock:          # un solo compresor a la vez
            with self._cam_lock:
                cached = self._cam_jpeg
            if cached is not None and cached[0] == seq:
                return cached                # lo ha hecho otro cliente
            try:
                jpeg = self._encode_jpeg(msg)
                self._cam_error_logged = False
            except Exception as e:  # noqa: BLE001 - imagen corrupta o formato raro
                if not self._cam_error_logged:
                    log.warning('Cámara: no se pudo comprimir la imagen (%s, %s)',
                                msg.encoding, e)
                    self._cam_error_logged = True
                return seq, None
            with self._cam_lock:
                self._cam_jpeg = (seq, jpeg)
            return seq, jpeg

    def _encode_jpeg(self, msg):
        h, w, enc, step = msg.height, msg.width, msg.encoding, msg.step
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, step)
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(enc)
        if channels is None:
            raise ValueError(f'codificación no soportada: {enc}')
        img = buf[:, :w * channels]
        if channels > 1:
            img = img.reshape(h, w, channels)
        conversion = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                      'bgra8': cv2.COLOR_BGRA2BGR}.get(enc)
        if conversion is not None:
            img = cv2.cvtColor(img, conversion)
        max_w = self.cfg.CAMERA_MAX_WIDTH
        if w > max_w:
            img = cv2.resize(img, (max_w, round(h * max_w / w)), interpolation=cv2.INTER_AREA)
        ok, out = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.cfg.CAMERA_JPEG_QUALITY])
        if not ok:
            raise ValueError('imencode falló')
        return out.tobytes()

    def get_state(self):
        with self._lock:
            pose = dict(self._pose) if self._pose else None
            nav = dict(self._nav)
            nav.pop('work', None)                       # datos internos del ajuste
            if nav.get('approach') is not None:
                nav['approach'] = dict(nav['approach'])
            if nav.get('route') is not None:
                route = dict(nav['route'])
                wait_until = route.pop('wait_until', None)
                if wait_until is not None:
                    route['wait_remaining'] = round(max(0.0, wait_until - time.monotonic()), 1)
                route['skipped'] = list(route['skipped'])
                route['stops'] = [{k: v for k, v in st.items() if k != 'work'}
                                  for st in route['stops']]
                nav['route'] = route
        return {'type': 'state',
                'ros_ok': self._ros_ok,
                'pose': pose,
                'nav': nav,
                'route_running': nav['state'] in NAV_RUNNING,
                'teleop_active': self._teleop_active,
                'nav2_ready': self.nav_client.server_is_ready(),
                'approach_ready': self.approach_ready(),
                'camera': self.camera_status(),
                'arm': self.arm.get_state(),
                'work_here': self._work_here_state()}

    def _work_here_state(self):
        with self._lock:
            w = self._work_here
        if w is None:
            return None
        return {'point_id': w['point_id'], 'name': w['name'],
                'screws': [{'row': sc['row'], 'col': sc['col'], 'z': sc['z'], 'u': sc['u']}
                           for sc in workgeo.screws(w['work']['wall'], w['work']['grid'])]}

    def get_config(self):
        """Límites que el frontend necesita para dibujar los controles."""
        c = self.cfg
        return {'type': 'config',
                'limits': {'vx_min': c.MIN_VX, 'vx_max': c.MAX_VX,
                           'vy_max': c.MAX_VY, 'wz_max': c.MAX_WZ},
                'base_frame': c.BASE_FRAME,
                'watchdog_timeout': c.WATCHDOG_TIMEOUT,
                'arm': {'joints': c.ARM_JOINTS,
                        'jog_max_vel': c.ARM_JOG_MAX_VEL,
                        'watchdog_timeout': c.ARM_WATCHDOG_TIMEOUT}}

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
        if moving:
            with self._lock:
                left, self._work_here = self._work_here, None
            if left is not None:
                log.info('Teleop: el robot deja de estar ajustado frente a %s', left['name'])
        if moving and self.arm.moving():
            self._reject(who, 'no se puede mover la base mientras el brazo se mueve')
        if moving and self.nav_running():
            log.info('Teleop de %s: se cancela la navegación en curso', who)
            self._emit('warning', 'nav_preempted',
                       'Teleoperación: se cancela la navegación en curso')
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
        self.arm.stop(who)

    def halt_teleop(self, who='?'):
        """Para la teleoperación en curso sin tocar la navegación (p. ej. al
        cambiar el control de operador)."""
        if self._teleop_active:
            log.info('Teleop detenida (%s)', who)
        self._publish_twist(0.0, 0.0, 0.0)
        self._teleop_active = False
        self.arm.stop(who)

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

    # ------------------------------------------------------------- navegación
    # Hay un único "trabajo" de navegación a la vez: un goal suelto o una ruta.
    # Cada trabajo nuevo incrementa self._job; los callbacks de ROS comprueban
    # su número y se ignoran si pertenecen a un trabajo ya sustituido.

    def _replace_running(self, who):
        """Si hay navegación en marcha, avisa de que se sustituye."""
        with self._lock:
            running = self._nav['state'] in NAV_RUNNING
            nav = self._nav
        if running:
            what = (f'la ruta {nav["route"]["name"]}' if nav.get('mode') == 'route'
                    else 'el ajuste en curso' if nav.get('mode') == 'approach'
                    else 'el goal anterior')
            log.info('Navegación sustituida por %s: se cancela %s', who, what)
            self._emit('info', 'nav_replaced', f'Se cancela {what} para iniciar una nueva navegación')

    def _new_job(self):
        """Cancela lo que esté en marcha y devuelve el número del trabajo nuevo.
        La base va a moverse: el robot deja de estar frente a la pared."""
        with self._lock:
            self._job += 1
            job = self._job
            handle, self._goal_handle = self._goal_handle, None
            timer, self._wait_timer = self._wait_timer, None
            self._work_here = None
        self.arm.planner.clear_wall()
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        if handle is not None:
            handle.cancel_goal_async()
        return job

    def _check_pose_args(self, who, x, y, yaw):
        for name, value in (('x', x), ('y', y), ('yaw', yaw)):
            if not math.isfinite(value):
                self._reject(who, f'{name} no es un número válido')

    def _check_nav2(self, who):
        if self.arm.moving():
            self._reject(who, 'no se puede navegar mientras el brazo se mueve')
        if not self.nav_client.server_is_ready():
            self._reject(who, 'Nav2 no está disponible (navigate_to_pose sin servidor)')

    def approach_ready(self):
        return self.approach_client is not None and self.approach_client.server_is_ready()

    def _check_approach(self, who):
        if self.approach_client is None:
            self._reject(who, 'el gateway no tiene kuka_interfaces: no puede ajustar puntos de trabajo')
        if not self.approach_client.server_is_ready():
            self._reject(who, 'el nodo de ajuste (wall_approach) no está en marcha')

    def send_goal(self, x, y, yaw, who='?', work=None, name=None, point_id=None):
        """Goal suelto. Con `work` ({wall, distance}) es un punto de trabajo: al
        llegar se lanza el acercamiento fino a la pared."""
        self._check_pose_args(who, x, y, yaw)
        self._check_nav2(who)
        if work:
            self._check_approach(who)
        self._replace_running(who)
        job = self._new_job()
        log.info('GOAL de %s: x=%.2f y=%.2f yaw=%.2f%s', who, x, y, yaw,
                 f' (punto de trabajo {name}: ajuste al llegar)' if work else '')
        with self._lock:
            self._nav = {'mode': 'goal', 'state': 'sending',
                         'goal': {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4)}}
            if name:
                self._nav['target'] = name
            if work:
                self._nav['work'] = work
                self._nav['phase'] = 'navigating'
                self._nav['point_id'] = point_id
        self._navigate(job, x, y, yaw)

    def approach(self, point, who='?'):
        """Solo el acercamiento fino frente a un punto de trabajo (botón «Ajustar»):
        el robot ya tiene que estar delante de la pared."""
        if point.get('kind') != 'work' or not point.get('work'):
            self._reject(who, f'{point.get("name")!r} no es un punto de trabajo')
        if self.arm.moving():
            self._reject(who, 'no se puede ajustar la base mientras el brazo se mueve')
        self._check_approach(who)
        self._replace_running(who)
        job = self._new_job()
        log.info('AJUSTE de %s frente a %s', who, point['name'])
        with self._lock:
            self._nav = {'mode': 'approach', 'state': 'sending', 'phase': 'adjusting',
                         'target': point['name'], 'work': point['work'], 'point_id': point['id'],
                         'goal': {'x': round(point['x'], 3), 'y': round(point['y'], 3),
                                  'yaw': round(point['yaw'], 4)}}
        self._approach(job, point['work'])

    def run_route(self, route, who='?'):
        """route: dict de la base de datos con 'id', 'name' y 'stops'
        (cada parada con name, x, y, yaw, wait_s)."""
        stops = route.get('stops') or []
        if not stops:
            self._reject(who, f'la ruta {route.get("name")!r} no tiene paradas')
        for s in stops:
            self._check_pose_args(who, s['x'], s['y'], s['yaw'])
        self._check_nav2(who)
        n_work = sum(1 for s in stops if s.get('kind') == 'work' and s.get('work'))
        if n_work:
            self._check_approach(who)
        self._replace_running(who)
        job = self._new_job()
        log.info('RUTA %s iniciada por %s: %d paradas', route['name'], who, len(stops))
        self._emit('info', 'route_started',
                   f'Ruta {route["name"]} iniciada ({len(stops)} paradas)')
        with self._lock:
            self._nav = {
                'mode': 'route', 'state': 'active', 'goal': None,
                'route': {
                    'id': route['id'], 'name': route['name'],
                    'total': len(stops), 'current': 0, 'phase': 'navigating',
                    'reached': 0, 'skipped': [],
                    'stops': [{'name': s['name'], 'x': s['x'], 'y': s['y'],
                               'yaw': s['yaw'], 'wait_s': s['wait_s'],
                               'kind': s.get('kind', 'move'), 'work': s.get('work'),
                               'point_id': s.get('point_id')}
                              for s in stops],
                },
            }
        self._route_next(job)

    # --- secuenciador de rutas
    def _route_next(self, job):
        """Lanza la navegación a la parada actual o termina la ruta."""
        with self._lock:
            if job != self._job:
                return
            r = self._nav['route']
            i = r['current']
            if i >= r['total']:
                done = True
            else:
                done = False
                stop = r['stops'][i]
                r['phase'] = 'navigating'
                r.pop('wait_until', None)
                self._nav.pop('approach', None)
                self._nav['goal'] = {'x': stop['x'], 'y': stop['y'], 'yaw': stop['yaw']}
                self._nav['state'] = 'active'
                self._nav.pop('distance_remaining', None)
        if done:
            self._route_finish(job)
            return
        log.info('RUTA %s: parada %d/%d (%s)', r['name'], i + 1, r['total'], stop['name'])
        self._navigate(job, stop['x'], stop['y'], stop['yaw'])

    def _route_stop_reached(self, job):
        with self._lock:
            if job != self._job:
                return
            r = self._nav['route']
            stop = r['stops'][r['current']]
            r['reached'] += 1
            wait = float(stop.get('wait_s') or 0.0)
            if wait > 0:
                r['phase'] = 'waiting'
                r['wait_until'] = time.monotonic() + wait
        log.info('RUTA %s: alcanzada %s, espera %.1f s', r['name'], stop['name'], wait)
        if wait > 0:
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            timer = self.create_timer(wait, lambda: self._route_wait_done(job), clock=steady)
            with self._lock:
                if job != self._job:          # cancelado mientras tanto
                    self.destroy_timer(timer)
                    return
                self._wait_timer = timer
        else:
            self._route_advance(job)

    def _route_wait_done(self, job):
        with self._lock:
            timer, self._wait_timer = self._wait_timer, None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        self._route_advance(job)

    def _route_advance(self, job):
        with self._lock:
            if job != self._job:
                return
            self._nav['route']['current'] += 1
        self._route_next(job)

    def _route_stop_failed(self, job, status_name, detail):
        with self._lock:
            if job != self._job:
                return
            r = self._nav['route']
            stop = r['stops'][r['current']]
            skip = self.cfg.ROUTE_ON_FAILURE == 'skip'
            if skip:
                r['skipped'].append(stop['name'])
        if skip:
            log.warning('RUTA %s: parada %s fallida (%s%s), se salta', r['name'],
                        stop['name'], status_name, detail)
            self._emit('warning', 'route_stop_failed',
                       f'Ruta {r["name"]}: no se alcanzó {stop["name"]}{detail}; se continúa')
            self._route_advance(job)
        else:
            log.warning('RUTA %s abortada en %s (%s%s)', r['name'], stop['name'],
                        status_name, detail)
            with self._lock:
                self._nav['state'] = 'aborted'
            self._emit('warning', 'route_failed',
                       f'Ruta {r["name"]} detenida: no se alcanzó {stop["name"]} '
                       f'(parada {r["current"] + 1}/{r["total"]}){detail}')

    def _route_finish(self, job):
        with self._lock:
            if job != self._job:
                return
            r = self._nav['route']
            self._nav['state'] = 'succeeded'
            self._nav['goal'] = None
            r.pop('wait_until', None)
            r['phase'] = 'done'
        summary = f'{r["reached"]}/{r["total"]} paradas'
        if r['skipped']:
            summary += f'; saltadas: {", ".join(r["skipped"])}'
        log.info('RUTA %s terminada: %s', r['name'], summary)
        self._emit('warning' if r['skipped'] else 'info', 'route_finished',
                   f'Ruta {r["name"]} terminada: {summary}')

    # --- acción NavigateToPose (común a goal y ruta)
    def _navigate(self, job, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.cfg.MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=lambda msg: self._on_feedback(job, msg))
        future.add_done_callback(lambda f: self._on_goal_response(job, f))

    def _on_goal_response(self, job, future):
        handle = future.result()
        with self._lock:
            stale = job != self._job
        if stale:
            if handle.accepted:
                handle.cancel_goal_async()    # llegó tarde: ya hay otro trabajo
            return
        if not handle.accepted:
            log.warning('Nav2 rechazó el goal')
            self._emit('warning', 'nav_rejected', 'Nav2 rechazó el goal')
            with self._lock:
                self._nav['state'] = 'rejected'
            return
        with self._lock:
            self._goal_handle = handle
            self._nav['state'] = 'active'
        handle.get_result_async().add_done_callback(lambda f: self._on_result(job, f))

    def _on_feedback(self, job, msg):
        with self._lock:
            if job == self._job and self._nav['state'] in NAV_RUNNING:
                self._nav['distance_remaining'] = round(msg.feedback.distance_remaining, 2)

    def _on_result(self, job, future):
        res = future.result()
        status = res.status
        name = GOAL_STATUS_NAMES.get(status, f'status_{status}')
        error_code = int(getattr(res.result, 'error_code', 0) or 0)
        error_msg = str(getattr(res.result, 'error_msg', '') or '')
        detail = f' — {error_msg}' if error_msg else (f' (código {error_code})' if error_code else '')

        with self._lock:
            if job != self._job:
                return
            self._goal_handle = None
            self._nav.pop('distance_remaining', None)
            mode = self._nav.get('mode')

        if mode == 'route':
            if status == GoalStatus.STATUS_SUCCEEDED:
                with self._lock:
                    r = self._nav['route']
                    stop = r['stops'][r['current']]
                if stop.get('kind') == 'work' and stop.get('work'):
                    log.info('RUTA %s: en %s, ajuste frente a la pared', r['name'], stop['name'])
                    self._approach(job, stop['work'])
                else:
                    self._route_stop_reached(job)
            elif status != GoalStatus.STATUS_CANCELED:
                self._route_stop_failed(job, name, detail)
            return

        # goal suelto a un punto de trabajo: al llegar, ajuste fino
        with self._lock:
            work = self._nav.get('work')
        if status == GoalStatus.STATUS_SUCCEEDED and work:
            log.info('Punto de trabajo alcanzado: ajuste frente a la pared')
            self._approach(job, work)
            return

        # goal suelto
        with self._lock:
            self._nav['state'] = name
            if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
                self._nav['goal'] = None
            if error_code:
                self._nav['error_code'] = error_code
            if error_msg:
                self._nav['error_msg'] = error_msg
        if status == GoalStatus.STATUS_SUCCEEDED:
            log.info('Navegación terminada: %s', name)
            self._emit('info', 'nav_succeeded', 'Goal alcanzado')
        elif status != GoalStatus.STATUS_CANCELED:
            log.warning('Navegación terminada: %s (error_code=%d %s)', name, error_code, error_msg)
            self._emit('warning', 'nav_failed',
                       f'Navegación fallida: {name}{detail}. El goal sigue marcado.')

    # --- acción ApproachWall (ajuste fino frente a la pared, 6.4)
    def _approach(self, job, work):
        """Lanza el acercamiento fino con los datos de un punto de trabajo."""
        if not self.approach_ready():                 # p. ej. el nodo se ha caído durante la ruta
            self._approach_done(job, False, 'el nodo de ajuste (wall_approach) no está en marcha', None)
            return
        (x1, y1), (x2, y2) = work['wall']
        goal = ApproachWall.Goal()
        goal.wall_p1.x, goal.wall_p1.y = float(x1), float(y1)
        goal.wall_p2.x, goal.wall_p2.y = float(x2), float(y2)
        goal.distance = float(work['distance'])
        goal.lateral = 0.0
        with self._lock:
            if job != self._job:
                return
            self._nav['state'] = 'active'
            self._nav.pop('distance_remaining', None)
            self._nav['approach'] = {'phase': 'iniciando'}
            if self._nav.get('mode') == 'route':
                self._nav['route']['phase'] = 'adjusting'
            else:
                self._nav['phase'] = 'adjusting'
        future = self.approach_client.send_goal_async(
            goal, feedback_callback=lambda msg: self._on_approach_feedback(job, msg))
        future.add_done_callback(lambda f: self._on_approach_response(job, f))

    def _on_approach_response(self, job, future):
        handle = future.result()
        with self._lock:
            stale = job != self._job
        if stale:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._approach_done(job, False, 'el nodo de ajuste rechazó la meta '
                                            '(¿ya hay otro ajuste en curso?)', None)
            return
        with self._lock:
            self._goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_approach_result(job, f))

    def _on_approach_feedback(self, job, msg):
        fb = msg.feedback
        with self._lock:
            if job != self._job or self._nav['state'] not in NAV_RUNNING:
                return
            self._nav['approach'] = {
                'phase': fb.phase,
                'distance_error': round(fb.distance_error, 4),
                'lateral_error': round(fb.lateral_error, 4),
                'angle_error': round(fb.angle_error, 4),
                'points': fb.wall_points,
            }

    def _on_approach_result(self, job, future):
        res = future.result()
        with self._lock:
            if job != self._job:
                return
            self._goal_handle = None
        if res.status == GoalStatus.STATUS_CANCELED:
            return                     # lo ha cancelado el gateway: cancel_goal ya lo gestiona
        r = res.result
        ok = res.status == GoalStatus.STATUS_SUCCEEDED and bool(r.success)
        finals = {'distance_error': round(r.final_distance_error, 4),
                  'lateral_error': round(r.final_lateral_error, 4),
                  'angle_error': round(r.final_angle_error, 4)}
        self._approach_done(job, ok, r.message or GOAL_STATUS_NAMES.get(res.status, ''), finals)

    def _approach_done(self, job, ok, message, finals):
        with self._lock:
            if job != self._job:
                return
            mode = self._nav.get('mode')
            a = dict(self._nav.get('approach') or {})
            if finals:
                a.update(finals)
            a['phase'] = 'hecho' if ok else 'fallido'
            a['message'] = message
            self._nav['approach'] = a
            if mode == 'route':
                stop = self._nav['route']['stops'][self._nav['route']['current']]
                target, work, point_id = stop['name'], stop.get('work'), stop.get('point_id')
            else:
                target = self._nav.get('target', 'el punto')
                work, point_id = self._nav.get('work'), self._nav.get('point_id')
            if ok and work:
                self._work_here = {'point_id': point_id, 'name': target, 'work': work}
        errs = (f' (d {finals["distance_error"] * 1000:+.0f} mm, lat {finals["lateral_error"] * 1000:+.0f} mm, '
                f'θ {math.degrees(finals["angle_error"]):+.1f}°)') if finals else ''
        if ok:
            log.info('AJUSTE en %s correcto%s', target, errs)
            if work:
                self.arm.planner.set_wall(target, work)
        else:
            log.warning('AJUSTE en %s fallido: %s%s', target, message, errs)

        if mode == 'route':
            if ok:
                self._route_stop_reached(job)
            else:
                self._route_stop_failed(job, 'ajuste', f' — ajuste fallido: {message}')
            return
        with self._lock:
            self._nav['state'] = 'succeeded' if ok else 'aborted'
            self._nav.pop('phase', None)
            if ok:
                self._nav['goal'] = None
        if ok:
            self._emit('info', 'approach_succeeded', f'Posicionado frente a {target}{errs}')
        else:
            self._emit('warning', 'approach_failed', f'Ajuste frente a {target} fallido: {message}{errs}')

    # --- brazo frente a la pared (5.3)
    def arm_to_screw(self, point_id, row, col, speed=None, who='?'):
        """Lleva el TCP delante de un tornillo de la rejilla del punto de trabajo
        frente al que está ajustado el robot."""
        with self._lock:
            w = self._work_here
        if w is None or w['point_id'] != point_id:
            self._reject(who, 'el robot no está ajustado frente a ese punto de trabajo: '
                              'usa «Ir» o «Ajustar» primero')
        screw = next((sc for sc in workgeo.screws(w['work']['wall'], w['work']['grid'])
                      if sc['row'] == row and sc['col'] == col), None)
        if screw is None:
            self._reject(who, f'el tornillo F{row + 1}·C{col + 1} no existe en la rejilla')
        # Objetivo en el frame de la base (perpendicular a la pared real, ver
        # planner.screw_tcp_pose_base) y de ahí, por TF, a la base del brazo.
        c = self.cfg
        pos_base, quat_base = plannerlib.screw_tcp_pose_base(
            w['work']['distance'], screw, c.SCREW_STANDOFF, c.BASE_HALF_LENGTH, c.ARM_OFFSET_Y,
            base_height=self.arm.planner.base_height())
        pos, quat = self.arm.planner.to_arm_base(c.BASE_FRAME, pos_base, quat_base)
        log.info('TORNILLO F%d·C%d de %s (%s): TCP en %s (%.3f, %.3f, %.3f), a %.2f m de la pared',
                 row + 1, col + 1, w['name'], who, c.BASE_FRAME, *pos_base, c.SCREW_STANDOFF)
        self.arm.move_tcp(pos, quat, who=who, speed=speed,
                          label=f'tornillo F{row + 1}·C{col + 1} de {w["name"]}', check=True)

    def nav_running(self):
        with self._lock:
            return self._nav['state'] in NAV_RUNNING

    def cancel_goal(self, who='?'):
        """Cancela el goal o la ruta en curso; si no hay nada en marcha, borra
        el goal o la ruta que hayan quedado marcados (p. ej. tras un fallo)."""
        with self._lock:
            running = self._nav['state'] in NAV_RUNNING
            nav = dict(self._nav)
        if running:
            self._new_job()
            with self._lock:
                self._nav['state'] = 'canceled'
                self._nav['goal'] = None
                self._nav.pop('distance_remaining', None)
                self._nav.pop('phase', None)
                if 'route' in self._nav:
                    self._nav['route'].pop('wait_until', None)
                    self._nav['route']['phase'] = 'done'
            if nav.get('mode') == 'route':
                r = nav['route']
                log.info('RUTA %s cancelada por %s en la parada %d/%d', r['name'], who,
                         r['current'] + 1, r['total'])
                self._emit('info', 'nav_canceled',
                           f'Ruta {r["name"]} cancelada en la parada {r["current"] + 1}/{r["total"]}')
            elif nav.get('mode') == 'approach' or nav.get('phase') == 'adjusting':
                log.info('Ajuste cancelado por %s', who)
                self._emit('info', 'nav_canceled', 'Ajuste frente a la pared cancelado')
            else:
                log.info('Navegación cancelada por %s', who)
                self._emit('info', 'nav_canceled', 'Navegación cancelada')
            return
        if nav.get('goal') is not None or nav.get('route') is not None:
            log.info('Goal/ruta borrados por %s', who)
            with self._lock:
                self._nav = {'mode': None, 'state': 'idle', 'goal': None}


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
