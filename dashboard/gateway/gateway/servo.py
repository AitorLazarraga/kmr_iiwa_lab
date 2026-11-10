"""MoveIt Servo desde el gateway (paso 5.4).

Servo (nodo servo_node de moveit_servo, lanzado en moveit_sim.launch.py) mueve
el brazo en tiempo real con comprobación de colisiones y singularidades y
publica en el topic del joint_trajectory_controller (mismo controlador que
MoveIt: no hay cambio de controlador en todo el ciclo).

Dos usos:
  - Jog cartesiano del TCP desde el dashboard (velocidades TwistStamped),
    en el frame de la herramienta o de la base del brazo. Hay que repetir la
    orden mientras se pulsa (10 Hz); si deja de llegar, el watchdog para.
  - Corrección corta del TCP (pose objetivo PoseStamped): cuando el check del
    TCP falla, Servo lleva el TCP a la pose pedida. Como mucho
    TCP_CORRECTION_ATTEMPTS intentos y solo si el error es menor que
    TCP_CORRECTION_MAX_POS / _ANGLE; si no, se para y se avisa.

Servo se mantiene en PAUSA (servicio ~/pause_servo) salvo mientras el gateway
lo usa, para que nunca interfiera con los planes de MoveIt.

Las consignas llevan el sello de tiempo del reloj del gateway (simulado si
USE_SIM_TIME): Servo descarta las que tienen más de incoming_command_timeout.
"""
import logging
import math
import threading
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from std_srvs.srv import SetBool
from tf2_ros import TransformException

from .errors import CommandRejected
from .planner import tcp_errors

try:
    from moveit_msgs.msg import ServoStatus
    from moveit_msgs.srv import ServoCommandType
except ImportError:  # pragma: no cover - moveit_msgs sin Servo (MoveIt antiguo)
    ServoStatus = ServoCommandType = None

log = logging.getLogger('gateway.servo')

TWIST, POSE = 1, 2                 # moveit_msgs/srv/ServoCommandType
STATUS_TEXT = {                     # moveit_servo StatusCode
    -1: 'orden no válida',
    0: '',
    1: 'frenando: cerca de una singularidad',
    2: 'PARADO: singularidad',
    3: 'frenando: saliendo de una singularidad',
    4: 'frenando: cerca de una colisión',
    5: 'PARADO: colisión',
    6: 'en el límite de una articulación',
}
FRAMES = ('tool', 'base')


class ServoController:
    def __init__(self, node, cfg, arm):
        self.node = node
        self.cfg = cfg
        self.arm = arm
        self._lock = threading.Lock()
        base = cfg.SERVO_NODE
        self._twist_pub = node.create_publisher(TwistStamped, f'{base}/delta_twist_cmds', 10)
        self._pose_pub = node.create_publisher(PoseStamped, f'{base}/pose_target_cmds', 10)
        self._pause_cli = node.create_client(SetBool, f'{base}/pause_servo')
        self._type_cli = (node.create_client(ServoCommandType, f'{base}/switch_command_type')
                          if ServoCommandType else None)
        if ServoStatus is not None:
            node.create_subscription(ServoStatus, f'{base}/status', self._on_status, 10)
        else:
            log.warning('moveit_msgs sin ServoStatus/ServoCommandType: Servo no disponible')

        # Estado
        self._state = 'unknown'     # unknown | paused | arming | twist | pose
        self._wanted = None         # 'twist' | 'pose' | None: lo que se está activando
        self._status = (0, '')
        self._status_time = 0.0
        # Jog cartesiano
        self._twist = [0.0] * 6
        self._twist_frame = 'tool'
        self._jog_last = 0.0
        self._jog_active = False
        # Corrección
        self._corr = None           # dict con la corrección en curso
        self._result = None         # último resultado de corrección (para la web)

        steady = Clock(clock_type=ClockType.STEADY_TIME)
        node.create_timer(1.0 / cfg.SERVO_RATE, self._tick, clock=steady)
        node.create_timer(1.0, self._keep_paused, clock=steady)

    # ------------------------------------------------------------ estado
    def ready(self):
        return (self._type_cli is not None and self._pause_cli.service_is_ready()
                and self._type_cli.service_is_ready())

    def active(self):
        with self._lock:
            return self._jog_active or self._corr is not None or self._state == 'arming'

    def get_state(self):
        with self._lock:
            corr = dict(self._corr) if self._corr else None
            if corr:
                for k in ('pos', 'quat', 'started', 'on_done', 'stable'):
                    corr.pop(k, None)              # internos (on_done es una función: no va a JSON)
            code, msg = self._status
            stale = time.monotonic() - self._status_time > 1.0
            return {
                'ready': self.ready(),
                'state': self._state,
                'jog_active': self._jog_active,
                'frame': self._twist_frame,
                'status': None if stale or code == 0 else {'code': code, 'text': STATUS_TEXT.get(code, msg)},
                'correction': corr,
                'last_correction': self._result,
                'max_linear': self.cfg.SERVO_MAX_LINEAR,
                'max_angular': self.cfg.SERVO_MAX_ANGULAR,
            }

    def _on_status(self, msg):
        with self._lock:
            prev = self._status[0]
            self._status = (int(msg.code), msg.message)
            self._status_time = time.monotonic()
        if msg.code in (2, 5) and prev != msg.code:
            text = STATUS_TEXT.get(int(msg.code), msg.message)
            log.warning('Servo: %s (%s)', text, msg.message)
            self.node.emit('warning', 'servo_status', f'Servo: {text}')

    # ------------------------------------------------------------ pausa / tipo de orden
    def _keep_paused(self):
        """Al conectar con Servo (arranca SIN pausa) o si se desconoce su estado,
        pausarlo. Así nunca está activo sin que el gateway lo use."""
        with self._lock:
            unknown = self._state == 'unknown'
        if unknown and self.ready():
            self._pause(True)

    def _pause(self, paused, then=None):
        req = SetBool.Request()
        req.data = bool(paused)
        future = self._pause_cli.call_async(req)

        def done(f):
            try:
                ok = f.result().success
            except Exception as e:  # noqa: BLE001
                ok = False
                log.error('Servo: error en pause_servo (%s)', e)
            if paused:
                with self._lock:
                    if self._wanted is None:
                        self._state = 'paused' if ok else 'unknown'
                if ok:
                    log.info('Servo en pausa')
            if then is not None:
                then(ok)
        future.add_done_callback(done)

    def _activate(self, kind, then):
        """Pone Servo en modo `kind` ('twist' | 'pose') y lo saca de la pausa;
        then(ok) al terminar. Asíncrono."""
        with self._lock:
            self._wanted = kind
            self._state = 'arming'
        req = ServoCommandType.Request()
        req.command_type = TWIST if kind == 'twist' else POSE
        future = self._type_cli.call_async(req)

        def typed(f):
            try:
                ok = f.result().success
            except Exception as e:  # noqa: BLE001
                ok = False
                log.error('Servo: error en switch_command_type (%s)', e)
            if not ok:
                self._armed(kind, False, then)
                return
            self._pause(False, lambda ok2: self._armed(kind, ok2, then))
        future.add_done_callback(typed)

    def _armed(self, kind, ok, then):
        with self._lock:
            if self._wanted != kind:          # entretanto se ha parado
                ok = False
            elif ok:
                self._state = kind
            else:
                self._state = 'unknown'
                self._wanted = None
        then(ok)

    def _release(self):
        """Fin de uso: pausa."""
        with self._lock:
            self._wanted = None
            self._state = 'paused' if self._state != 'unknown' else 'unknown'
        if self.ready():
            self._pause(True)

    # ------------------------------------------------------------ comprobaciones
    def _check_can_use(self, who):
        if not self.ready():
            raise CommandRejected('MoveIt Servo no está en marcha (servo_node en moveit_sim.launch.py)')
        if self.node.nav_running():
            raise CommandRejected('no se puede mover el brazo mientras la base navega o se desplaza')
        if self.arm.joint_positions() is None:
            raise CommandRejected('sin joint_states recientes')
        mode = self.arm.mode()
        if mode != 'trajectory':
            # Servo publica en el joint_trajectory_controller: hay que activarlo
            if mode != 'switching':
                self.arm.set_mode('trajectory', who)
            raise CommandRejected('activando el modo trayectoria del brazo: repite la orden en un momento')

    def _stamp(self):
        return self.node.get_clock().now().to_msg()

    # ------------------------------------------------------------ jog cartesiano
    def jog(self, v, frame, who='?'):
        """v = [vx, vy, vz, wx, wy, wz] en m/s y rad/s, en el frame `frame`
        ('tool' = TCP, 'base' = base del brazo). Repetir a 10 Hz."""
        c = self.cfg
        if frame not in FRAMES:
            raise CommandRejected("frame debe ser 'tool' o 'base'")
        if not isinstance(v, (list, tuple)) or len(v) != 6:
            raise CommandRejected('arm_cart necesita 6 velocidades [vx, vy, vz, wx, wy, wz]')
        try:
            v = [float(x) for x in v]
        except (TypeError, ValueError):
            raise CommandRejected('las velocidades deben ser números')
        if not all(math.isfinite(x) for x in v):
            raise CommandRejected('velocidad no válida (NaN o infinito)')
        if math.hypot(*v[:3]) > c.SERVO_MAX_LINEAR + 1e-9:
            raise CommandRejected(f'velocidad lineal por encima de {c.SERVO_MAX_LINEAR * 100:.0f} cm/s')
        if math.hypot(*v[3:]) > c.SERVO_MAX_ANGULAR + 1e-9:
            raise CommandRejected(f'velocidad angular por encima de {math.degrees(c.SERVO_MAX_ANGULAR):.0f}°/s')
        moving = any(abs(x) > 1e-9 for x in v)
        if not moving:
            self._stop_jog()
            return
        with self._lock:
            corr = self._corr is not None
        if corr:
            raise CommandRejected('hay una corrección del TCP en curso')
        self._check_can_use(who)
        # El jog cartesiano interrumpe cualquier movimiento planificado o jog articular
        self.arm.stop_for_servo(who)
        with self._lock:
            self._twist = v
            self._twist_frame = frame
            self._jog_last = time.monotonic()
            started = not self._jog_active
            self._jog_active = True
            state = self._state
        if started:
            log.info('Servo: jog cartesiano INICIO (%s, frame %s)', who, frame)
        if state not in ('twist', 'arming'):
            self._activate('twist', lambda ok: None if ok else self._jog_failed())

    def _jog_failed(self):
        with self._lock:
            self._jog_active = False
        self.node.emit('warning', 'servo', 'Servo: no se pudo activar el jog cartesiano')

    def _stop_jog(self, reason=None):
        with self._lock:
            was = self._jog_active
            self._jog_active = False
            self._twist = [0.0] * 6
        if was:
            if reason:
                log.warning('Servo: jog cartesiano detenido (%s)', reason)
            # Unos ciclos de velocidad cero para que Servo frene suave y después pausa
            for _ in range(3):
                self._publish_twist([0.0] * 6, self._twist_frame)
            self._release()

    def _publish_twist(self, v, frame):
        msg = TwistStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.cfg.ARM_TCP_FRAME if frame == 'tool' else self.cfg.ARM_BASE_FRAME
        (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
         msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z) = v
        self._twist_pub.publish(msg)

    # ------------------------------------------------------------ corrección del TCP
    def correct(self, pos, quat, label, who='gateway', on_done=None):
        """Lleva el TCP a (pos, quat) en ARM_BASE_FRAME con Servo (modo pose).
        Rechaza si el error es mayor que el corregible. on_done(ok, text)."""
        c = self.cfg
        err = self._tcp_error(pos, quat)
        if err is None:
            raise CommandRejected('sin TF del TCP: no se puede corregir')
        pos_err, ang_err = err
        if pos_err > c.TCP_CORRECTION_MAX_POS or ang_err > c.TCP_CORRECTION_MAX_ANGLE:
            raise CommandRejected(
                f'error de {pos_err * 1000:.1f} mm · {ang_err:.1f}°: más de lo corregible con Servo '
                f'({c.TCP_CORRECTION_MAX_POS * 1000:.0f} mm · {c.TCP_CORRECTION_MAX_ANGLE:.0f}°). '
                'Usa el botón del tornillo (LIN) o vuelve a ajustar')
        with self._lock:
            if self._corr is not None:
                raise CommandRejected('ya hay una corrección del TCP en curso')
            jog = self._jog_active
        if jog:
            raise CommandRejected('hay un jog cartesiano en curso')
        self._check_can_use(who)
        self.arm.stop_for_servo(who)
        with self._lock:
            self._corr = {'label': label, 'attempt': 1, 'attempts': c.TCP_CORRECTION_ATTEMPTS,
                          'pos': tuple(pos), 'quat': tuple(quat), 'started': None, 'stable': 0,
                          'pos_error': round(pos_err, 4), 'angle_error': round(ang_err, 2),
                          'state': 'arming', 'on_done': on_done}
            self._result = None
        log.info('Servo: corrección de %s (%s): %.1f mm · %.2f°', label, who, pos_err * 1000, ang_err)
        self.node.emit('info', 'servo_correction',
                       f'Corrigiendo el TCP en {label} con Servo ({pos_err * 1000:.1f} mm · {ang_err:.1f}°)')
        self._activate('pose', self._corr_armed)

    def _corr_armed(self, ok):
        with self._lock:
            corr = self._corr
            if corr is None:
                return
            if ok:
                corr['state'] = 'tracking'
                corr['started'] = time.monotonic()
                corr['stable'] = 0
        if not ok:
            self._corr_finish(False, 'no se pudo activar Servo en modo pose')

    def _tcp_error(self, pos, quat):
        try:
            tf = self.node.tf_buffer.lookup_transform(self.cfg.ARM_BASE_FRAME, self.cfg.ARM_TCP_FRAME, Time())
        except TransformException:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return tcp_errors((t.x, t.y, t.z), (r.x, r.y, r.z, r.w), pos, quat)

    def _publish_pose(self, pos, quat):
        msg = PoseStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.cfg.ARM_BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = quat
        self._pose_pub.publish(msg)

    def _corr_tick(self, corr):
        c = self.cfg
        self._publish_pose(corr['pos'], corr['quat'])
        err = self._tcp_error(corr['pos'], corr['quat'])
        if err is None:
            return
        pos_err, ang_err = err
        within = pos_err <= c.TCP_CHECK_POS_TOL and ang_err <= c.TCP_CHECK_ANGLE_TOL
        with self._lock:
            if self._corr is not corr:
                return
            corr['pos_error'] = round(pos_err, 4)
            corr['angle_error'] = round(ang_err, 2)
            corr['stable'] = corr['stable'] + 1 if within else 0
            stable = corr['stable']
            elapsed = time.monotonic() - corr['started']
            attempt = corr['attempt']
        if stable >= c.SERVO_SETTLE_CYCLES:
            self._corr_finish(True, f'{pos_err * 1000:.1f} mm · {ang_err:.2f}° en el intento {attempt}')
            return
        if elapsed > c.SERVO_CORRECTION_TIMEOUT:
            if attempt >= c.TCP_CORRECTION_ATTEMPTS:
                self._corr_finish(False, f'{attempt} intentos sin llegar a la tolerancia '
                                         f'(queda {pos_err * 1000:.1f} mm · {ang_err:.2f}°)')
                return
            # Siguiente intento: se reinicia Servo (pausa y vuelta a activar)
            log.info('Servo: corrección, intento %d sin llegar (%.1f mm · %.2f°)', attempt,
                     pos_err * 1000, ang_err)
            with self._lock:
                corr['attempt'] += 1
                corr['state'] = 'arming'
            self._pause(True, lambda ok: self._activate('pose', self._corr_armed))

    def _corr_finish(self, ok, text):
        with self._lock:
            corr, self._corr = self._corr, None
            if corr is None:
                return
            self._result = {'label': corr['label'], 'ok': ok, 'text': text}
        self._release()
        if ok:
            log.info('Servo: corrección de %s correcta: %s', corr['label'], text)
            self.node.emit('info', 'servo_correction', f'TCP corregido en {corr["label"]}: {text}')
        else:
            log.warning('Servo: corrección de %s fallida: %s', corr['label'], text)
            self.node.emit('warning', 'servo_correction',
                           f'No se ha podido corregir el TCP en {corr["label"]}: {text}. Brazo parado')
        if corr.get('on_done'):
            corr['on_done'](ok, text)

    # ------------------------------------------------------------ bucle y parada
    def _tick(self):
        with self._lock:
            state = self._state
            jog = self._jog_active
            silence = time.monotonic() - self._jog_last
            v, frame = list(self._twist), self._twist_frame
            corr = self._corr
            tracking = corr is not None and corr['state'] == 'tracking'
        if jog:
            if silence > self.cfg.ARM_WATCHDOG_TIMEOUT:
                self._stop_jog(f'{silence:.2f} s sin órdenes')
                self.node.emit('warning', 'arm_watchdog',
                               f'Brazo: jog cartesiano interrumpido ({silence:.2f} s sin órdenes)')
                return
            if state == 'twist':
                self._publish_twist(v, frame)
            return
        if tracking and state == 'pose':
            if self.node.nav_running():
                self._corr_finish(False, 'la base ha empezado a moverse')
                return
            self._corr_tick(corr)

    def stop(self, who='?'):
        """PARADA, pérdida de control u otro movimiento del brazo: para el jog
        cartesiano y cancela la corrección. Servo queda en pausa."""
        self._stop_jog(f'parado por {who}')
        with self._lock:
            corr = self._corr is not None
        if corr:
            self._corr_finish(False, f'cancelada por {who}')
