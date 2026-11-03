"""Brazo LBR iiwa 14: estado, límites, pose del TCP, jog articular y movimientos
planificados con MoveIt.

ArmController no es un nodo propio: usa el nodo del gateway (RosBridge) para
crear sus suscripciones, publicadores, servicios y temporizadores.

Modos (según el controlador activo):
  'jog'        -> forward_position_controller: jog articular integrado a 50 Hz
  'trajectory' -> joint_trajectory_controller: lo usa MoveIt para ejecutar

Movimientos planificados (5.3): ir a una postura e ir delante de un tornillo se
piden a MoveIt (planner.py), que comprueba colisiones con la plataforma, el
propio brazo y la pared del punto de trabajo. Si MoveIt no está en marcha se
rechazan; el jog articular sigue siendo directo.
"""
import itertools
import logging
import math
import threading
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformException

from .errors import CommandRejected
from .planner import MoveItPlanner, error_text, tcp_errors

log = logging.getLogger('gateway.arm')

MODE_TEXT = {'jog': 'jog articular', 'trajectory': 'trayectoria'}
MOVE_RUNNING = ('pending', 'sending', 'planning', 'active')
MOVEIT_SUCCESS = 1          # moveit_msgs/MoveItErrorCodes.SUCCESS


def duration_msg(seconds):
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int((seconds - sec) * 1e9))

def rpy_from_quat(q):
    """Cuaternión -> (roll, pitch, yaw) en rad, convención ZYX (la de URDF y tf2)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    # Por redondeo sinp puede pasar un poco de ±1 y asin daría error
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw

class ArmController():
    def __init__(self, node, cfg):
        self.node = node
        self.cfg = cfg
        self._lock = threading.Lock()
        
        self._tcp = None
        node.create_timer(0.1, self._update_tcp)
        self._positions = None  
        self._velocities = None
        self._joints_time = 0.0
        self._missing_logged = False 

        node.create_subscription(JointState, cfg.JOINT_STATES_TOPIC,
                                 self._on_joint_states, 10)

        log.info('Brazo: escuchando %s (%d articulaciones)',
                 cfg.JOINT_STATES_TOPIC, len(cfg.ARM_JOINTS))

        self._limits = None           # dict por articulación, o None hasta recibir el URDF

        description_qos = QoSProfile(depth=1,
                                     reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, cfg.ROBOT_DESCRIPTION_TOPIC,
                                 self._on_robot_description, description_qos)

        self._mode = 'unknown'
        self._controllers = {'jog': cfg.ARM_JOG_CONTROLLER,
                             'trajectory': cfg.ARM_TRAJ_CONTROLLER}
        self._jog_target = None

        cm = cfg.CONTROLLER_MANAGER
        self._list_cli = node.create_client(ListControllers, f'{cm}/list_controllers')
        self._switch_cli = node.create_client(SwitchController, f'{cm}/switch_controller')
        self._cmd_pub = node.create_publisher(Float64MultiArray, cfg.ARM_POSITION_CMD_TOPIC, 10)

        self._jog_vel = [0.0] * 7      # rad/s pedidas por el navegador
        self._jog_last = 0.0           # time.monotonic() de la última orden de jog
        self._jog_active = False
        self._jog_tick_time = None     # para calcular dt real entre ticks
        self._at_limit = set() 
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        node.create_timer(1.0 / cfg.ARM_JOG_RATE, self._jog_tick, clock=steady)

        # Movimientos planificados con MoveIt
        self.planner = MoveItPlanner(node, cfg)
        self._move = None              # estado del movimiento planificado (dict) o None
        self._move_handle = None
        self._move_job = itertools.count(1)
        self._move_current = 0         # trabajo vigente; los callbacks viejos se ignoran
        self._pending_move = None      # movimiento a lanzar cuando termine el cambio de modo
        self._tcp_target = None        # (pos, quat) en ARM_BASE_FRAME si hay que comprobar el TCP al llegar
        self._on_done = {}             # job -> función(ok, reason) a llamar al terminar (reposo antes de mover la base)

        node.create_timer(2.0, self._refresh_mode)

    def _refresh_mode(self):
        with self._lock:
            if self._mode == 'switching':
                return                     
        if not self._list_cli.service_is_ready():
            return                        
        future = self._list_cli.call_async(ListControllers.Request())
        future.add_done_callback(self._on_list)

    def _on_list(self, future):
        try:
            res = future.result()
        except Exception as e:  
            log.warning('Brazo: no se pudo consultar los controladores (%s)', e)
            return
        states = {c.name: c.state for c in res.controller}   
        if states.get(self._controllers['jog']) == 'active':
            mode = 'jog'
        elif states.get(self._controllers['trajectory']) == 'active':
            mode = 'trajectory'
        else:
            mode = 'none'

        with self._lock:
            if self._mode == 'switching':
                return                    
            previous, self._mode = self._mode, mode
        if mode != previous:
            log.info('Brazo: modo detectado %s -> %s', previous, mode)
            if mode == 'jog':
                self._seed_jog()            

    def _update_tcp(self):
        try:
            tf = self.node.tf_buffer.lookup_transform(
                self.cfg.ARM_BASE_FRAME, self.cfg.ARM_TCP_FRAME, Time())
        except TransformException:
            return                 
        
        t = tf.transform.translation
        q = tf.transform.rotation

        roll, pitch, yaw = rpy_from_quat(q)

        tcp = {
            'frame': self.cfg.ARM_BASE_FRAME,
            'x': round(t.x, 4), 'y': round(t.y, 4), 'z': round(t.z, 4),
            'roll': round(roll, 4), 'pitch': round(pitch, 4), 'yaw': round(yaw, 4),
            'quat': [round(q.x, 5), round(q.y, 5), round(q.z, 5), round(q.w, 5)],
        }

        with self._lock:
            self._tcp = tcp

    def _on_joint_states(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}

        # ¿Están las 7 articulaciones del brazo?
        missing = [j for j in self.cfg.ARM_JOINTS if j not in index]
        if missing:
            if not self._missing_logged:
                log.warning('Brazo: joint_states sin %s; se ignora el mensaje', missing)
                self._missing_logged = True
            return
        self._missing_logged = False

        positions = [float(msg.position[index[j]]) for j in self.cfg.ARM_JOINTS]

        # La velocidad puede venir vacía según el publicador
        if len(msg.velocity) == len(msg.name):
            velocities = [float(msg.velocity[index[j]]) for j in self.cfg.ARM_JOINTS]
        else:
            velocities = None

        with self._lock:
            self._positions = positions
            self._velocities = velocities
            self._joints_time = time.monotonic()

    def _on_robot_description(self, msg):
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as e:
            log.error(f'Brazo: Robot_description XML no valido {e}')
            return

        joints = {j.get('name'): j for j in root.findall('joint')}
        limits = {}

        for name in self.cfg.ARM_JOINTS:
            joint = joints.get(name)
            limit = joint.find('limit') if joint is not None else None

            if limit is None:
                log.error('Brazo: el URDF no tiene <limit> para %s; el jog queda deshabilitado', name)
                return
            
            lower = float(limit.get('lower'))
            upper = float(limit.get('upper'))
            velocity = float(limit.get('velocity'))

            soft_lower = lower + self.cfg.ARM_LIMIT_MARGIN
            soft_upper = upper - self.cfg.ARM_LIMIT_MARGIN

            if name in self.cfg.ARM_LIMITS_OVERRIDE_DEG:
                o_low, o_up = (math.radians(v) for v in self.cfg.ARM_LIMITS_OVERRIDE_DEG[name])
                soft_lower = max(soft_lower, o_low)
                soft_upper = min(soft_upper, o_up)

            if soft_lower >= soft_upper:
                log.error('Brazo: límites blandos imposibles en %s (%.1f° >= %.1f°)',
                          name, math.degrees(soft_lower), math.degrees(soft_upper))
                return

            limits[name] = {
                'lower': lower, 'upper': upper,
                'soft_lower': soft_lower, 'soft_upper': soft_upper,
                'velocity': velocity,
            }

        with self._lock:
            self._limits = limits

        log.info('Brazo: límites leídos del URDF (blandos, en grados):')
        for name, lim in limits.items():
            log.info('  %-12s [%7.1f, %7.1f]  vmax %5.1f °/s', name,
                     math.degrees(lim['soft_lower']), math.degrees(lim['soft_upper']),
                     math.degrees(lim['velocity']))

    def get_limits(self):
        with self._lock:
            return {k: dict(v) for k, v in self._limits.items()} if self._limits else None

    def _limits_for_web(self):
        lim = self.get_limits()
        if lim is None:
            return None
        return [{'soft_lower': round(lim[j]['soft_lower'], 4),
                 'soft_upper': round(lim[j]['soft_upper'], 4),
                 'velocity': round(lim[j]['velocity'], 4)} for j in self.cfg.ARM_JOINTS]

    def joint_positions(self):
        with self._lock:
            if self._positions is None:
                return None
            if time.monotonic() - self._joints_time > self.cfg.ARM_STATE_TIMEOUT:
                return None
            return list(self._positions)

    def get_state(self):
        with self._lock:
            age = time.monotonic() - self._joints_time if self._joints_time else None
            positions = list(self._positions) if self._positions else None
            velocities = list(self._velocities) if self._velocities else None
            tcp = dict(self._tcp) if self._tcp else None
            mode = self._mode
            jog_active = self._jog_active
            move = dict(self._move) if self._move else None
        if move is not None:
            started = move.pop('started', None)
            if started is not None and move['state'] == 'active':
                move['elapsed'] = round(time.monotonic() - started, 1)
        ok = age is not None and age < self.cfg.ARM_STATE_TIMEOUT
        return {
            'ok': ok,
            'joints': self.cfg.ARM_JOINTS,
            'positions': [round(p, 4) for p in positions] if ok else None,
            'velocities': [round(v, 4) for v in velocities] if ok and velocities else None,
            'tcp': tcp if ok else None,
            'mode': mode if ok else None,
            'limits': self._limits_for_web(),
            'jog_active': jog_active,
            'move': move,
            'moveit_ready': self.planner.ready(),
            'speeds': list(self.cfg.MOVEIT_SPEEDS),
            'default_speed': self.cfg.MOVEIT_DEFAULT_SPEED,
        }

    def _seed_jog(self):
        """Fija el objetivo del jog en la posición real y la envía al controlador,
        para que al activarlo el brazo se quede exactamente donde está."""
        positions = self.joint_positions()
        if positions is None:
            log.warning('Brazo: modo jog sin joint_states recientes; no se envía posición inicial')
            return
        with self._lock:
            self._jog_target = list(positions)
        self._cmd_pub.publish(Float64MultiArray(data=positions))
        log.info('Brazo: jog inicializado en la posición actual')

    def set_mode(self, mode, who='?'):
        """Pide activar el controlador del modo indicado. Asíncrono: el resultado
        llega en _on_switch. Lanza CommandRejected si no se puede pedir."""
        if mode not in self._controllers:
            raise CommandRejected(f'modo de brazo desconocido: {mode!r}')
        with self._lock:
            current = self._mode
        if current == mode:
            return
        if current == 'switching':
            raise CommandRejected('ya hay un cambio de modo del brazo en curso')
        if current == 'unknown':
            self._refresh_mode()
            raise CommandRejected('estado de los controladores aún desconocido; reintenta en un momento')
        if not self._switch_cli.service_is_ready():
            raise CommandRejected('controller_manager no disponible')
        if self.joint_positions() is None:
            raise CommandRejected('sin joint_states recientes: no se puede cambiar de modo con seguridad')

        req = SwitchController.Request()
        req.activate_controllers = [self._controllers[mode]]
        # Solo se desactiva el que está activo: en modo STRICT, pedir desactivar
        # uno que ya está inactivo hace fallar todo el cambio.
        req.deactivate_controllers = [self._controllers[current]] if current in self._controllers else []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=2)

        with self._lock:
            self._mode = 'switching'
        log.info('Brazo: cambio de modo %s -> %s (%s)', current, mode, who)
        future = self._switch_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_switch(f, current, mode, who))

    def _on_switch(self, future, previous, mode, who):
        try:
            ok = future.result().ok
        except Exception as e:  # noqa: BLE001
            ok = False
            log.error('Brazo: error en el cambio de controlador (%s)', e)
        if not ok:
            with self._lock:
                self._mode = 'unknown'          # el refresco periódico dirá dónde estamos
                pending, self._pending_move = self._pending_move, None
                if pending is not None:
                    self._move = {**self._move, 'state': 'aborted'}
            log.error('Brazo: el controller_manager rechazó el cambio %s -> %s', previous, mode)
            self.node.emit('warning', 'arm_mode', f'No se pudo cambiar el brazo a modo {mode}')
            if pending is not None:
                self._notify_done(pending[0], False, f'no se pudo activar el modo {mode}')
            return
        with self._lock:
            self._mode = mode
            pending, self._pending_move = self._pending_move, None
        if mode == 'jog':
            self._seed_jog()
        log.info('Brazo: modo %s activo', mode)
        self.node.emit('info', 'arm_mode', f'Brazo en modo {MODE_TEXT[mode]}')
        if pending is not None:
            if mode == 'trajectory':
                job, launch = pending
                with self._lock:
                    current = job == self._move_current
                if current:
                    launch(job)
            else:
                with self._lock:
                    self._move = {**self._move, 'state': 'canceled'}
                self._notify_done(pending[0], False, 'cancelado')

    def jog(self, velocities, who='?'):
        """Orden de jog: 7 velocidades en rad/s. Hay que repetirla mientras se
        quiera mover (10 Hz); si deja de llegar, el watchdog para el brazo."""

        if not isinstance(velocities, (list, tuple)) or len(velocities) != len(self.cfg.ARM_JOINTS):
            raise CommandRejected(f'arm_jog necesita {len(self.cfg.ARM_JOINTS)} velocidades')
        try:
            v = [float(x) for x in velocities]
        except (TypeError, ValueError):
            raise CommandRejected('las velocidades deben ser números')
        if not all(math.isfinite(x) for x in v):
            raise CommandRejected('velocidad no válida (NaN o infinito)')

        vmax = self.cfg.ARM_JOG_MAX_VEL
        for name, x in zip(self.cfg.ARM_JOINTS, v):
            if abs(x) > vmax + 1e-9:
                log.warning('RECHAZADO jog de %s: %s a %.1f°/s (máx %.1f°/s)',
                            who, name, math.degrees(x), math.degrees(vmax))
                raise CommandRejected(f'{name}: {math.degrees(x):.1f}°/s supera el máximo '
                                      f'de jog ({math.degrees(vmax):.0f}°/s)')

        if self.node.nav_running():
            raise CommandRejected('no se puede mover el brazo mientras la base navega')
        if self.get_limits() is None:
            raise CommandRejected('límites del brazo aún no recibidos (robot_description)')
        if self.joint_positions() is None:
            raise CommandRejected('sin joint_states recientes')

        # Un jog interrumpe cualquier movimiento a postura en curso
        self.cancel_move(who, quiet=True)

        with self._lock:
            mode = self._mode
        if mode == 'switching':
            return
        if mode != 'jog':
            self.set_mode('jog', who)
            return

        moving = any(abs(x) > 1e-6 for x in v)
        with self._lock:
            started = moving and not self._jog_active
            self._jog_vel = v
            self._jog_last = time.monotonic()
            self._jog_active = moving
        if started:
            log.info('Brazo: jog INICIO (%s)', who)


    def _jog_tick(self):
        now = time.monotonic()
        dt = 0.0 if self._jog_tick_time is None else min(now - self._jog_tick_time, 0.1)
        self._jog_tick_time = now

        with self._lock:
            if self._mode != 'jog' or self._jog_target is None or not self._jog_active:
                return
            silence = now - self._jog_last
            v = list(self._jog_vel)
            target = list(self._jog_target)

        # Watchdog: el navegador dejó de mandar órdenes
        if silence > self.cfg.ARM_WATCHDOG_TIMEOUT:
            with self._lock:
                self._jog_active = False
            log.warning('Brazo: WATCHDOG, %.2f s sin órdenes de jog -> parado', silence)
            self.node.emit('warning', 'arm_watchdog',
                           f'Brazo: jog interrumpido ({silence:.2f} s sin órdenes)')
            return                       # el controlador mantiene la última posición

        positions = self.joint_positions()
        limits = self.get_limits()
        if positions is None or limits is None:
            with self._lock:
                self._jog_active = False
            log.error('Brazo: jog detenido, sin joint_states o sin límites')
            self.node.emit('warning', 'arm_jog', 'Brazo: jog detenido (sin estado del brazo)')
            return

        for i, name in enumerate(self.cfg.ARM_JOINTS):
            if v[i] == 0.0:
                continue
            new = target[i] + v[i] * dt

            # Límites blandos
            lim = limits[name]
            if new <= lim['soft_lower'] or new >= lim['soft_upper']:
                new = min(max(new, lim['soft_lower']), lim['soft_upper'])
                if name not in self._at_limit:
                    self._at_limit.add(name)
                    log.warning('Brazo: %s en el límite (%.1f°)', name, math.degrees(new))
                    self.node.emit('warning', 'arm_limit', f'{name[-2:]} en el límite')
            else:
                self._at_limit.discard(name)

            # Seguimiento: no dejar que el objetivo se escape de la posición real
            error = new - positions[i]
            if abs(error) > self.cfg.ARM_MAX_TRACKING_ERROR and (error > 0) == (v[i] > 0):
                continue                 # mantener target[i]: esperar a que el brazo llegue

            target[i] = new

        with self._lock:
            self._jog_target = target
        self._cmd_pub.publish(Float64MultiArray(data=target))

    def stop(self, who='?'):
        """PARADA o pérdida de control: deja el brazo quieto donde está
        (para el jog y cancela cualquier movimiento a postura)."""
        with self._lock:
            was_active = self._jog_active
            self._jog_active = False
            self._jog_vel = [0.0] * 7
            mode = self._mode
        if mode == 'jog':
            self._seed_jog()              # objetivo = posición real, y se publica
        if was_active:
            log.warning('Brazo: jog detenido por %s', who)
        self.cancel_move(who)

    def moving(self):
        """True si el brazo se está moviendo por orden del gateway (jog o postura)."""
        with self._lock:
            return self._jog_active or (self._move is not None and self._move['state'] in MOVE_RUNNING)

    # ------------------------------------------------------------ movimientos planificados
    def _check_speed(self, speed):
        if speed is None:
            return self.cfg.MOVEIT_DEFAULT_SPEED
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            raise CommandRejected('velocidad no válida')
        if not any(abs(speed - s) < 1e-6 for s in self.cfg.MOVEIT_SPEEDS):
            raise CommandRejected(f'velocidad {speed:.2f} no permitida '
                                  f'({", ".join(f"{s:.0%}" for s in self.cfg.MOVEIT_SPEEDS)})')
        return speed

    def _check_can_move(self, internal=False):
        if not internal and self.node.nav_running():
            raise CommandRejected('no se puede mover el brazo mientras la base navega')
        if self.joint_positions() is None:
            raise CommandRejected('sin joint_states recientes')
        self.planner.check_ready()

    def move_to(self, positions, who='?', label=None, speed=None, internal=False, on_done=None):
        """Lleva el brazo a 7 ángulos (rad, orden A1..A7) con un plan de MoveIt
        (sin colisiones). Rechaza si MoveIt no está en marcha.

        internal=True: lo pide el propio gateway como parte de un movimiento de
        la base (llevar el brazo a reposo antes de navegar), así que no se
        rechaza por haber una navegación en marcha. on_done(ok, reason) se llama
        una sola vez cuando el movimiento termina, falla o se cancela."""
        if not isinstance(positions, (list, tuple)) or len(positions) != len(self.cfg.ARM_JOINTS):
            raise CommandRejected(f'la postura necesita {len(self.cfg.ARM_JOINTS)} ángulos')
        try:
            target = [float(p) for p in positions]
        except (TypeError, ValueError):
            raise CommandRejected('los ángulos deben ser números')
        if not all(math.isfinite(p) for p in target):
            raise CommandRejected('ángulo no válido (NaN o infinito)')
        limits = self.get_limits()
        if limits is None:
            raise CommandRejected('límites del brazo no disponibles')
        for name, p in zip(self.cfg.ARM_JOINTS, target):
            lim = limits[name]
            if not lim['soft_lower'] <= p <= lim['soft_upper']:
                raise CommandRejected(
                    f'{name}: {math.degrees(p):.1f}° fuera de los límites '
                    f'[{math.degrees(lim["soft_lower"]):.0f}°, {math.degrees(lim["soft_upper"]):.0f}°]')
        if not internal:
            speed = self._check_speed(speed)
        self._check_can_move(internal=internal)
        goal = self.planner.joint_goal(self.cfg.ARM_JOINTS, target, speed)
        self._start_move(goal, who, label or 'postura', speed, target=[round(p, 4) for p in target],
                         on_done=on_done)

    def move_tcp(self, pos, quat, who='?', label=None, speed=None, check=False):
        """Lleva el TCP a una pose (en ARM_BASE_FRAME) con un plan de MoveIt.
        Con check=True, al terminar se ejecuta check_tcp_orientation."""
        speed = self._check_speed(speed)
        self._check_can_move()
        goal = self.planner.pose_goal(pos, quat, speed)
        tcp_target = (tuple(pos), tuple(quat)) if check else None
        self._start_move(goal, who, label or 'pose del TCP', speed, tcp_target=tcp_target)

    def _start_move(self, goal, who, label, speed, target=None, tcp_target=None, on_done=None):
        """Común a postura y pose: para el jog, cancela lo anterior, pasa a modo
        trayectoria si hace falta y manda el objetivo a MoveIt."""
        with self._lock:
            self._jog_active = False
            self._jog_vel = [0.0] * 7
            mode = self._mode
        self.cancel_move(who, quiet=True)
        if mode == 'switching':
            raise CommandRejected('cambio de modo del brazo en curso; reintenta')

        job = next(self._move_job)
        launch = lambda j: self._send_moveit(j, goal, label)   # noqa: E731
        with self._lock:
            self._move_current = job
            self._move = {'state': 'pending' if mode != 'trajectory' else 'sending',
                          'label': label, 'speed': speed}
            if target is not None:
                self._move['target'] = target
            self._tcp_target = tcp_target
            if on_done is not None:
                self._on_done[job] = on_done
        log.info('Brazo: ir a %s con MoveIt al %d %% (%s)', label, round(speed * 100), who)
        if mode == 'trajectory':
            launch(job)
        else:
            with self._lock:
                self._pending_move = (job, launch)
            try:
                self.set_mode('trajectory', who)
            except CommandRejected:
                with self._lock:
                    self._pending_move = None
                    self._move = None
                    self._on_done.pop(job, None)     # quien llamó recibe la excepción
                raise

    def _notify_done(self, job, ok, reason=None):
        """Avisa (una sola vez) a quien pidió el movimiento `job` con on_done."""
        with self._lock:
            fn = self._on_done.pop(job, None)
        if fn is not None:
            fn(ok, reason)

    def _send_moveit(self, job, goal, label):
        with self._lock:
            if job != self._move_current:
                return                          # sustituido mientras cambiaba el modo
            self._move = {**self._move, 'state': 'planning'}
        future = self.planner.send(goal, lambda msg: self._on_moveit_feedback(job, msg))
        future.add_done_callback(lambda f: self._on_move_response(job, f, label))

    def _on_moveit_feedback(self, job, msg):
        state = str(getattr(msg.feedback, 'state', '') or '').upper()
        with self._lock:
            if job != self._move_current or self._move is None:
                return
            if state == 'MONITOR' and self._move['state'] != 'active':
                self._move = {**self._move, 'state': 'active', 'started': time.monotonic()}
            elif state == 'PLANNING':
                self._move = {**self._move, 'state': 'planning'}

    def _on_move_response(self, job, future, label):
        handle = future.result()
        with self._lock:
            stale = job != self._move_current
        if stale:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            with self._lock:
                self._move = {**self._move, 'state': 'rejected'}
            log.warning('Brazo: MoveIt rechazó la petición')
            self.node.emit('warning', 'arm_move', 'Brazo: MoveIt rechazó la petición')
            self._notify_done(job, False, 'MoveIt rechazó la petición')
            return
        with self._lock:
            self._move_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_move_result(job, f, label))

    def _on_move_result(self, job, future, label):
        res = future.result()
        with self._lock:
            if job != self._move_current:
                return
            self._move_handle = None
        code = int(getattr(getattr(res.result, 'error_code', None), 'val', 0) or 0)
        if res.status == GoalStatus.STATUS_SUCCEEDED and code == MOVEIT_SUCCESS:
            state, reason = 'succeeded', None
            log.info('Brazo: movimiento a %s completado', label)
            self.node.emit('info', 'arm_move', f'Brazo: movimiento a {label} completado')
        elif res.status == GoalStatus.STATUS_CANCELED:
            state, reason = 'canceled', None
        else:
            state, reason = 'aborted', error_text(code)
            log.warning('Brazo: movimiento a %s fallido (status %d, código MoveIt %d: %s)',
                        label, res.status, code, reason)
            self.node.emit('warning', 'arm_move', f'Brazo: no se alcanzó {label} — {reason}')
        with self._lock:
            self._move = {k: v for k, v in self._move.items() if k != 'started'}
            self._move['state'] = state
            if reason:
                self._move['reason'] = reason
            target = self._tcp_target if state == 'succeeded' else None
            if target is not None:
                self._move['check'] = {'state': 'checking'}
        self._notify_done(job, state == 'succeeded', reason or (None if state == 'succeeded' else state))
        if target is not None:
            # Se espera un poco para que TF refleje la posición final del brazo
            timer = threading.Timer(self.cfg.TCP_CHECK_DELAY,
                                    lambda: self.check_tcp_orientation(job, label, *target))
            timer.daemon = True
            timer.start()

    # ------------------------------------------------------------ check del TCP
    def check_tcp_now(self, pos, quat, label):
        """check_tcp_orientation sin movimiento del brazo: tras un desplazamiento
        lateral de la base con la herramienta delante de un tornillo, comprueba
        que ha quedado delante del tornillo de la columna destino."""
        with self._lock:
            job = next(self._move_job)
            self._move_current = job
            self._move = {'state': 'succeeded', 'label': label, 'check': {'state': 'checking'}}
        timer = threading.Timer(self.cfg.TCP_CHECK_DELAY,
                                lambda: self.check_tcp_orientation(job, label, pos, quat))
        timer.daemon = True
        timer.start()

    def at_positions(self, target, tol_rad):
        """True si las 7 articulaciones están a menos de tol_rad de `target`."""
        positions = self.joint_positions()
        if positions is None:
            return False
        return all(abs(p - t) <= tol_rad for p, t in zip(positions, target))

    def check_tcp_orientation(self, job, label, target_pos, target_quat):
        """Compara por TF el TCP real con el pedido (posición y eje de la
        herramienta; el giro sobre su propio eje no cuenta). Deja el resultado en
        move['check'] y avisa en el dashboard si está fuera de tolerancia.
        Por ahora solo verifica: la corrección corta llegará con Servo (5.4)."""
        cfg = self.cfg
        level, text = None, None
        try:
            tf = self.node.tf_buffer.lookup_transform(cfg.ARM_BASE_FRAME, cfg.ARM_TCP_FRAME, Time())
        except TransformException as e:
            check = {'state': 'error', 'message': f'sin TF del TCP ({e})'}
            level, text = 'warning', f'Brazo: no se pudo comprobar el TCP en {label}: {check["message"]}'
        else:
            t, r = tf.transform.translation, tf.transform.rotation
            pos_err, ang_err = tcp_errors((t.x, t.y, t.z), (r.x, r.y, r.z, r.w),
                                          target_pos, target_quat)
            ok = pos_err <= cfg.TCP_CHECK_POS_TOL and ang_err <= cfg.TCP_CHECK_ANGLE_TOL
            check = {'state': 'ok' if ok else 'out',
                     'pos_error': round(pos_err, 4), 'angle_error': round(ang_err, 2),
                     'pos_tol': cfg.TCP_CHECK_POS_TOL, 'angle_tol': cfg.TCP_CHECK_ANGLE_TOL}
            errs = f'{pos_err * 1000:.1f} mm · {ang_err:.2f}°'
            if ok:
                level, text = 'info', f'Brazo: TCP verificado en {label} ({errs})'
            else:
                level, text = 'warning', (f'Brazo: TCP fuera de tolerancia en {label}: {errs} '
                                          f'(máx {cfg.TCP_CHECK_POS_TOL * 1000:.0f} mm · '
                                          f'{cfg.TCP_CHECK_ANGLE_TOL:.0f}°)')
        with self._lock:
            if job != self._move_current or self._move is None:
                return                  # ya hay otro movimiento: este check no vale
            self._move = {**self._move, 'check': check}
        (log.info if level == 'info' else log.warning)(text)
        self.node.emit(level, 'arm_check', text)

    def cancel_move(self, who='?', quiet=False):
        """Cancela el movimiento a postura en curso (si lo hay)."""
        with self._lock:
            running = self._move is not None and self._move['state'] in MOVE_RUNNING
            if not running:
                return
            old_job = self._move_current
            self._move_current = next(self._move_job)     # invalida callbacks pendientes
            handle, self._move_handle = self._move_handle, None
            self._pending_move = None
            label = self._move.get('label')
            self._move = {k: v for k, v in self._move.items() if k != 'started'}
            self._move['state'] = 'canceled'
        if handle is not None:
            handle.cancel_goal_async()      # el controlador se queda en la posición actual
        self._notify_done(old_job, False, 'cancelado')
        if not quiet:
            log.info('Brazo: movimiento a %s cancelado por %s', label or 'postura', who)
            self.node.emit('info', 'arm_move', f'Brazo: movimiento a {label or "postura"} cancelado')
    