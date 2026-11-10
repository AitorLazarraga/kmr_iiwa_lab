"""Mapas (paso 2.6): guardar el mapa de slam_toolbox, fijar la posición inicial
y listar los mapas guardados.

- Guardar (solo en modo mapeo). Quien escribe los ficheros es slam_toolbox, en
  su carpeta de mapas (cfg.MAPS_DIR, /maps en Docker). Se llaman dos servicios,
  uno detrás de otro:
    1. serialize_map -> <nombre>.posegraph + <nombre>.data   (los que usa la localización)
    2. save_map      -> <nombre>.pgm + <nombre>.yaml         (imagen, por si se usa map_server)
  Si falla el 1 el mapa NO se ha guardado. Si solo falla el 2, el mapa sirve
  igual para localizar y se avisa.
  Sobrescribir un mapa existente: solo un administrador (las dos confirmaciones
  las pide el dashboard; aquí se exige el rol y overwrite=True).
- Fijar posición (solo en localización): publica /initialpose y slam_toolbox
  se relocaliza alrededor de esa pose. Las comprobaciones de que la base esté
  parada las hace ros_bridge.set_initial_pose.
- Listar: lee cfg.MAPS_DIR (en Docker, el gateway la monta en solo lectura).
"""
import logging
import math
import os
import re
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.clock import Clock, ClockType

from .errors import CommandRejected

# Servicios de slam_toolbox. Si el paquete no está en el entorno, el resto del
# gateway funciona y «Guardar mapa» se rechaza con un mensaje claro.
try:
    from slam_toolbox.srv import SaveMap, SerializePoseGraph
except ImportError:  # pragma: no cover
    SaveMap = SerializePoseGraph = None

log = logging.getLogger('gateway.maps')

# Nombre de mapa: se usa tal cual como nombre de fichero y como argumento de
# start_sim.sh, así que nada de rutas, espacios ni mayúsculas.
NAME_RE = re.compile(r'^[a-z0-9][a-z0-9_-]{0,39}$')
NAME_RULE = 'minúsculas, números, «-» y «_», empezando por letra o número (máx. 40)'


def check_name(name):
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise CommandRejected(f'nombre de mapa no válido: {NAME_RULE}')
    return name


class MapManager:

    def __init__(self, node, cfg):
        self.node = node
        self.cfg = cfg
        self._lock = threading.Lock()
        self._job = 0
        # Guardado en curso: {job, name, step: 'serialize'|'image', started (time.time()),
        #                     step_started (monotonic), who, overwrite}
        self._saving = None
        self._last = None            # último guardado: {name, ok, message, time}

        self.pose_pub = node.create_publisher(PoseWithCovarianceStamped, cfg.INITIAL_POSE_TOPIC, 10)
        if SerializePoseGraph is not None:
            self.serialize_cli = node.create_client(SerializePoseGraph, cfg.SLAM_SERIALIZE_SERVICE)
            self.save_cli = node.create_client(SaveMap, cfg.SLAM_SAVE_MAP_SERVICE)
        else:
            self.serialize_cli = self.save_cli = None
            log.warning('slam_toolbox no disponible en el entorno: no se podrá guardar el mapa')
        # Vigilancia del tiempo de respuesta, con reloj de pared
        node.create_timer(1.0, self._check_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))
        log.info('Mapas: modo %s, mapa %s, carpeta %s', cfg.SLAM_MODE, cfg.MAP_NAME, cfg.MAPS_DIR)

    # ------------------------------------------------------------------ lista
    def list_maps(self):
        """Mapas utilizables para localizar (los que tienen .posegraph), o None si
        la carpeta no existe. size en bytes (.posegraph + .data)."""
        d = self.cfg.MAPS_DIR
        if not os.path.isdir(d):
            return None
        maps = []
        for f in sorted(os.listdir(d)):
            if not f.endswith('.posegraph'):
                continue
            name = f[:-len('.posegraph')]
            base = os.path.join(d, name)
            try:
                st = os.stat(base + '.posegraph')
            except OSError:
                continue
            data = base + '.data'
            maps.append({'name': name,
                         'modified': st.st_mtime,
                         'size': st.st_size + (os.path.getsize(data) if os.path.isfile(data) else 0),
                         'complete': os.path.isfile(data),       # sin .data no se puede cargar
                         'image': os.path.isfile(base + '.pgm')})
        return maps

    def exists(self, name):
        base = os.path.join(self.cfg.MAPS_DIR, name)
        return any(os.path.exists(base + ext) for ext in ('.posegraph', '.data'))

    # ---------------------------------------------------------------- guardar
    def save(self, name, overwrite=False, role='operator', who='?'):
        c = self.cfg
        if c.SLAM_MODE != 'mapping':
            raise CommandRejected('el mapa solo se guarda en modo mapeo '
                                  f'({c.START_CMD} mapping <nombre>)')
        check_name(name)
        if self.serialize_cli is None:
            raise CommandRejected('slam_toolbox no está en el entorno del gateway')
        if not os.path.isdir(c.MAPS_DIR):
            raise CommandRejected(f'no existe la carpeta de mapas {c.MAPS_DIR} (MAPS_DIR)')
        if not self.serialize_cli.service_is_ready():
            raise CommandRejected(f'slam_toolbox no responde ({c.SLAM_SERIALIZE_SERVICE} sin servidor)')
        exists = self.exists(name)
        if exists and not overwrite:
            raise CommandRejected(f'el mapa «{name}» ya existe')
        if exists and role != 'admin':
            raise CommandRejected('solo un administrador puede sobrescribir un mapa')
        with self._lock:
            if self._saving is not None:
                raise CommandRejected(f'ya se está guardando el mapa «{self._saving["name"]}»')
            self._job += 1
            job = self._job
            self._saving = {'job': job, 'name': name, 'step': 'serialize', 'started': time.time(),
                            'step_started': time.monotonic(), 'who': who, 'overwrite': exists}

        verb = 'Sobrescribiendo' if exists else 'Guardando'
        log.info('MAPA: %s «%s» (%s)', verb.lower(), name, who)
        self.node.emit('info' if not exists else 'warning', 'map_saving',
                       f'{verb} el mapa «{name}» ({who})')
        req = SerializePoseGraph.Request()
        req.filename = os.path.join(c.MAPS_DIR, name)
        future = self.serialize_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_serialized(job, f))

    def _current(self, job):
        with self._lock:
            s = self._saving
            return s if s is not None and s['job'] == job else None

    def _on_serialized(self, job, future):
        s = self._current(job)
        if s is None:                         # ya terminado (p. ej. por tiempo)
            return
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001
            self._finish(job, False, f'serialize_map ha fallado ({e})')
            return
        ok_code = getattr(SerializePoseGraph.Response, 'RESULT_SUCCESS', 0)
        code = getattr(resp, 'result', ok_code) if resp is not None else None
        if code != ok_code:
            self._finish(job, False, f'slam_toolbox no ha podido escribir el mapa (código {code})')
            return
        # slam_toolbox dice que sí: comprobar que los ficheros están y son nuevos
        base = os.path.join(self.cfg.MAPS_DIR, s['name'])
        try:
            fresh = all(os.path.getmtime(base + ext) >= s['started'] - 2.0
                        for ext in ('.posegraph', '.data'))
        except OSError:
            fresh = False
        if not fresh:
            self._finish(job, False, f'slam_toolbox no da error pero no aparece {base}.posegraph/.data '
                                     '(¿MAPS_DIR distinta en el gateway y en slam_toolbox?)')
            return

        # Paso 2: imagen .pgm/.yaml (opcional)
        if self.save_cli is None or not self.save_cli.service_is_ready():
            self._finish(job, True, 'sin imagen .pgm (save_map no disponible)', warn=True)
            return
        with self._lock:
            s['step'] = 'image'
            s['step_started'] = time.monotonic()
        req = SaveMap.Request()
        if hasattr(req.name, 'data'):         # std_msgs/String name
            req.name.data = base
        else:
            req.name = base
        future = self.save_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_image(job, f))

    def _on_image(self, job, future):
        if self._current(job) is None:
            return
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001
            self._finish(job, True, f'sin imagen .pgm (save_map ha fallado: {e})', warn=True)
            return
        ok_code = getattr(SaveMap.Response, 'RESULT_SUCCESS', 0)
        code = getattr(resp, 'result', ok_code) if resp is not None else None
        if code != ok_code:
            self._finish(job, True, f'sin imagen .pgm (save_map: código {code})', warn=True)
            return
        self._finish(job, True, '')

    def _finish(self, job, ok, detail, warn=False):
        with self._lock:
            s = self._saving
            if s is None or s['job'] != job:
                return
            self._saving = None
            name = s['name']
            if ok:
                message = f'Mapa «{name}» guardado' + (f' — {detail}' if detail else '')
            else:
                message = f'El mapa «{name}» NO se ha guardado: {detail}'
            self._last = {'name': name, 'ok': ok, 'message': message, 'time': time.time()}
        if ok:
            log.info('MAPA: %s', message)
            # 'maps_changed' -> los dashboards recargan la lista de mapas
            self.node.emit('warning' if warn else 'info', 'maps_changed', message)
        else:
            log.warning('MAPA: %s', message)
            self.node.emit('warning', 'map_save_failed', message)

    def _check_timeout(self):
        with self._lock:
            s = self._saving
            late = s is not None and time.monotonic() - s['step_started'] > self.cfg.MAP_SAVE_TIMEOUT
        if not late:
            return
        if s['step'] == 'serialize':
            self._finish(s['job'], False, f'slam_toolbox no ha respondido en {self.cfg.MAP_SAVE_TIMEOUT:.0f} s')
        else:
            self._finish(s['job'], True, 'sin imagen .pgm (save_map no ha respondido)', warn=True)

    # --------------------------------------------------------- posición inicial
    def publish_initial_pose(self, x, y, yaw):
        """/initialpose en el frame del mapa, con la incertidumbre de config."""
        c = self.cfg
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = c.MAP_FRAME
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = c.INITIAL_POSE_VAR_XY        # x, y
        cov[35] = c.INITIAL_POSE_VAR_YAW               # giro
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)

    # ------------------------------------------------------------------ estado
    def get_state(self):
        with self._lock:
            s = self._saving
            saving = ({'name': s['name'], 'step': s['step'], 'overwrite': s['overwrite'],
                       'elapsed': round(time.time() - s['started'], 1)} if s else None)
            last = dict(self._last) if self._last else None
        return {'mode': self.cfg.SLAM_MODE, 'map': self.cfg.MAP_NAME,
                'saving': saving, 'last': last}
