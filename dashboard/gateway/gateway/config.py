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

MAP_FRAME = _env('MAP_FRAME', 'map')
BASE_FRAME = _env('BASE_FRAME', 'kmp_base_link')

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

# --- Log --------------------------------------------------------------------
LOG_FILE = _env('LOG_FILE', 'logs/gateway.log')

# --- Frontend ---------------------------------------------------------------
# Durante el desarrollo el propio gateway sirve la web (en el paso 2.6 lo hará
# nginx). Por defecto: dashboard/web, al lado de dashboard/gateway.
WEB_DIR = _env('WEB_DIR', os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..', 'web')))
