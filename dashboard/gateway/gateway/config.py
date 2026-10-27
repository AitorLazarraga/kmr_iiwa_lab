"""Configuración del gateway.

Todo se puede sobrescribir con variables de entorno, para que en Docker
(paso 2.6) baste con un archivo .env.
"""
import math
import os


def _env(name, default):
    return os.environ.get(name, default)


def _env_float(name, default):
    return float(os.environ.get(name, default))


def _env_bool(name, default):
    return os.environ.get(name, str(default)).lower() in ('1', 'true', 'yes')


# --- Servidor web ---------------------------------------------------------
# 127.0.0.1: solo accesible desde la propia máquina (nginx hará de puerta).
# Para probar desde el móvil antes de tener nginx: GATEWAY_HOST=0.0.0.0
HOST = _env('GATEWAY_HOST', '127.0.0.1')
PORT = int(_env('GATEWAY_PORT', '8000'))

# --- ROS ------------------------------------------------------------------
USE_SIM_TIME = _env_bool('USE_SIM_TIME', True)

# Por dónde entra la teleoperación. Por defecto a la entrada del velocity
# smoother de Nav2: así pasa por el suavizado y por el collision_monitor
# antes de llegar a /kmr_iiwa/cmd_vel. Sin Nav2 lanzado, usar
# TELEOP_TOPIC=/kmr_iiwa/cmd_vel (directo al controlador, sin protección).
TELEOP_TOPIC = _env('TELEOP_TOPIC', '/cmd_vel_nav')
SCAN_TOPIC = _env('SCAN_TOPIC', '/kmr_iiwa/scan')
MAP_TOPIC = _env('MAP_TOPIC', '/map')
NAV_ACTION = _env('NAV_ACTION', '/navigate_to_pose')

CAMERA_TOPIC = _env('CAMERA_TOPIC', '/kmr_iiwa/arm_camera/image')

MAP_FRAME = _env('MAP_FRAME', 'map')
BASE_FRAME = _env('BASE_FRAME', 'kmp_base_link')

# Nombre del mapa con el que trabaja el gateway. Los puntos y rutas guardados
# pertenecen a un mapa: si se carga otro mapa en Nav2, lanzar con MAP_NAME=otro.
MAP_NAME = _env('MAP_NAME', 'laboratorio')

# Qué hacer si una parada de una ruta no se alcanza:
#   'stop' -> la ruta se detiene ahí (recomendado para atornillar: no seguir
#             si algo ha fallado)
#   'skip' -> se salta esa parada y se continúa con la siguiente
ROUTE_ON_FAILURE = _env('ROUTE_ON_FAILURE', 'stop')

# --- Brazo (Fase 4) -----------------------------------------------------------
ARM_JOINTS = [f'kmr_iiwa_A{i}' for i in range(1, 8)]          # A1 ... A7, en este orden
JOINT_STATES_TOPIC = _env('JOINT_STATES_TOPIC', '/kmr_iiwa/joint_states')
ROBOT_DESCRIPTION_TOPIC = _env('ROBOT_DESCRIPTION_TOPIC', '/kmr_iiwa/robot_description')
# Si no llegan joint_states en este tiempo, el estado del brazo no es fiable.
ARM_STATE_TIMEOUT = _env_float('ARM_STATE_TIMEOUT', 1.0)       # s

# Margen respecto a los límites del URDF: el brazo nunca pasa de
# (límite inferior + margen) a (límite superior - margen).
ARM_LIMIT_MARGIN = math.radians(_env_float('ARM_LIMIT_MARGIN_DEG', 5.0))
# Límites más estrechos por articulación (en grados). Solo ESTRECHAN, nunca amplían.
# Ejemplo: {'kmr_iiwa_A2': (-90, 90)}
ARM_LIMITS_OVERRIDE_DEG = {}

ARM_BASE_FRAME = _env('ARM_BASE_FRAME', 'kmr_iiwa_link_0')
ARM_TCP_FRAME = _env('ARM_TCP_FRAME', 'kmr_iiwa_tool_tcp')      # punta del atornillador (tool.xacro)

CONTROLLER_MANAGER = _env('CONTROLLER_MANAGER', '/kmr_iiwa/controller_manager')
ARM_JOG_CONTROLLER = _env('ARM_JOG_CONTROLLER', 'forward_position_controller')
ARM_TRAJ_CONTROLLER = _env('ARM_TRAJ_CONTROLLER', 'joint_trajectory_controller')
ARM_POSITION_CMD_TOPIC = _env('ARM_POSITION_CMD_TOPIC',
                              '/kmr_iiwa/forward_position_controller/commands')
ARM_TRAJ_ACTION = _env('ARM_TRAJ_ACTION',
                       '/kmr_iiwa/joint_trajectory_controller/follow_joint_trajectory')

# Jog articular
ARM_JOG_MAX_VEL = math.radians(_env_float('ARM_JOG_MAX_VEL_DEG', 15.0))   # rad/s por articulación
ARM_JOG_RATE = _env_float('ARM_JOG_RATE', 50.0)                           # Hz del integrador
ARM_WATCHDOG_TIMEOUT = _env_float('ARM_WATCHDOG_TIMEOUT', 0.3)            # s sin órdenes -> parar
# Máxima distancia entre objetivo y posición real antes de dejar de avanzar
ARM_MAX_TRACKING_ERROR = math.radians(_env_float('ARM_MAX_TRACKING_ERROR_DEG', 6.0))

# Ir a postura (trayectoria): velocidad máxima de la articulación que más se mueve
ARM_MOVE_MAX_VEL = math.radians(_env_float('ARM_MOVE_MAX_VEL_DEG', 20.0))  # rad/s
ARM_MOVE_MIN_TIME = _env_float('ARM_MOVE_MIN_TIME', 1.5)                    # s

# --- Puntos de trabajo (Fase 6.3) ---------------------------------------------
# Distancia por defecto del frontal del robot a la cara de la pared. Con la
# herramienta perpendicular a la pared, el análisis de alcance da ~1,0 m.
WORK_DEFAULT_DISTANCE = _env_float('WORK_DEFAULT_DISTANCE', 1.0)        # m
WORK_MAX_HEIGHT = _env_float('WORK_MAX_HEIGHT', 1.8)                    # m, fila superior
BASE_HALF_LENGTH = 0.54        # m: del centro de la KMP 200 a su frontal
ARM_OFFSET_Y = -0.1842         # m: el brazo está a la derecha del centro (eje Y de la base)

# --- Seguridad del movimiento ---------------------------------------------
# Límites de teleoperación. Una orden por encima se RECHAZA (no se recorta).
MAX_VX = _env_float('MAX_VX', 0.5)    # m/s
# El robot no admite marcha atrás: vx por debajo de este valor se rechaza.
MIN_VX = _env_float('MIN_VX', 0.0)    # m/s
MAX_VY = _env_float('MAX_VY', 0.5)    # m/s
MAX_WZ = _env_float('MAX_WZ', 1.0)    # rad/s

# Si no llega una orden de teleoperación en este tiempo, se manda parada.
WATCHDOG_TIMEOUT = _env_float('WATCHDOG_TIMEOUT', 0.3)   # s

# Si la pose del robot (TF map -> base) lleva más de esto sin actualizarse,
# se considera que la conexión con ROS se ha perdido.
ROS_STALE_TIMEOUT = _env_float('ROS_STALE_TIMEOUT', 1.0)  # s

# --- Envío de estado al navegador -----------------------------------------
STATE_RATE = _env_float('STATE_RATE', 10.0)   # Hz: pose y estado de navegación
SCAN_EVERY = int(_env('SCAN_EVERY', '2'))     # 1 de cada N envíos lleva scan
SCAN_DOWNSAMPLE = int(_env('SCAN_DOWNSAMPLE', '3'))  # 1 de cada N haces

# --- Cámara del brazo ---------------------------------------------------------
# El vídeo va por HTTP (MJPEG en /api/camera/arm.mjpg), no por el WebSocket, para
# no retrasar el estado ni las órdenes. Se comprime una vez por fotograma, solo
# mientras alguien la está viendo.
CAMERA_FPS = _env_float('CAMERA_FPS', 10.0)              # máximo enviado al navegador
CAMERA_JPEG_QUALITY = int(_env('CAMERA_JPEG_QUALITY', '70'))
CAMERA_MAX_WIDTH = int(_env('CAMERA_MAX_WIDTH', '640'))  # se reduce si llega más grande
CAMERA_STALE_TIMEOUT = _env_float('CAMERA_STALE_TIMEOUT', 1.0)  # s sin imagen = sin cámara

# --- Log --------------------------------------------------------------------
LOG_FILE = _env('LOG_FILE', 'logs/gateway.log')

# --- Usuarios y control -----------------------------------------------------
# Base de datos SQLite (no se sube a Git: está en data/, ignorado).
DB_PATH = _env('DB_PATH', 'data/gateway.db')
SESSION_HOURS = _env_float('SESSION_HOURS', 12.0)
# Con nginx + HTTPS (paso 2.6) poner COOKIE_SECURE=true.
COOKIE_SECURE = _env_bool('COOKIE_SECURE', False)
# Intentos de login fallidos permitidos por IP en LOGIN_WINDOW segundos.
LOGIN_MAX_FAILS = int(_env('LOGIN_MAX_FAILS', '5'))
LOGIN_WINDOW = _env_float('LOGIN_WINDOW', 60.0)
# Tiempo que se reserva el control a un operador desconectado para que pueda
# recuperarlo con la misma sesión (p. ej. corte de WiFi).
CONTROL_GRACE = _env_float('CONTROL_GRACE', 5.0)

# --- Frontend ---------------------------------------------------------------
# Durante el desarrollo el propio gateway sirve la web (en el paso 2.6 lo hará
# nginx). Por defecto: dashboard/web, al lado de dashboard/gateway.
WEB_DIR = _env('WEB_DIR', os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..', 'web')))
