"""Configuración del gateway.

Todo se puede sobrescribir con variables de entorno, para que en Docker
(paso 2.6) baste con un archivo .env.
"""
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
