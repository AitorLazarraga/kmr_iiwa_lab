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
# Acercamiento fino a la pared (paso 6.4): acción del nodo kuka_nav/wall_approach.py
APPROACH_ACTION = _env('APPROACH_ACTION', '/kmr_iiwa/approach_wall')
# Desplazamiento lateral entre columnas (acción ShiftAlongWall del mismo nodo, 6.5)
SHIFT_ACTION = _env('SHIFT_ACTION', '/kmr_iiwa/shift_along_wall')

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

# --- MoveIt (paso 5.3) ----------------------------------------------------------
# Los movimientos "ir a postura" e "ir a tornillo" se planifican con MoveIt
# (comprobando colisiones). Si move_group no está en marcha, se rechazan.
MOVEIT_ACTION = _env('MOVEIT_ACTION', '/kmr_iiwa/move_action')
MOVEIT_SCENE_SERVICE = _env('MOVEIT_SCENE_SERVICE', '/kmr_iiwa/apply_planning_scene')
MOVEIT_GROUP = _env('MOVEIT_GROUP', 'arm')
MOVEIT_PLANNING_TIME = _env_float('MOVEIT_PLANNING_TIME', 5.0)        # s por intento
# Velocidad (fracción de la máxima de cada articulación) que se puede elegir en
# el dashboard, y la de partida.
MOVEIT_SPEEDS = (0.10, 0.25, 0.50)
MOVEIT_DEFAULT_SPEED = 0.25
# Ir a tornillo: el TCP (cara del portapuntas) se coloca a esta distancia de la
# cara de la pared, con la herramienta perpendicular. El tornillo sobresale
# 5-8 cm del TCP, así que su punta queda a unos 4-7 cm de la pared.
SCREW_STANDOFF = _env_float('SCREW_STANDOFF', 0.12)                    # m
# Caja que representa la pared del punto de trabajo en la escena de MoveIt
WORK_WALL_THICKNESS = 0.10      # m, hacia dentro desde la cara marcada
WORK_WALL_HEIGHT = 2.5          # m desde el suelo
# Tolerancias que se piden a MoveIt para la pose del TCP. Tienen que ser más
# estrictas que las del check (abajo): si no, MoveIt puede dar por bueno un
# objetivo que luego el check marca como fuera de tolerancia.
MOVEIT_POSITION_TOL = 0.001     # m (radio de la esfera de la restricción de posición)
MOVEIT_AXIS_TOL = 0.01          # rad (≈0,6°) en cada eje de la orientación de la herramienta

# --- Postura de reposo -------------------------------------------------------
# Antes de cualquier movimiento NO lateral de la base (Nav2: «Ir», goal, rutas;
# «Ajustar»; reajuste tras un desplazamiento fallido) el brazo tiene que estar
# en la postura guardada con este nombre. Si no lo está, el gateway lo lleva con
# MoveIt y después mueve la base; si no puede, la base no se mueve.
# Solo un administrador puede crear, modificar o borrar esa postura.
# El desplazamiento lateral (ShiftAlongWall) y la teleop no la exigen.
ARM_SAFE_POSE_NAME = 'reposo'   # se compara sin mayúsculas ni espacios a los lados
ARM_SAFE_TOL_DEG = 2.0          # ° por articulación para considerar que ya está en reposo
ARM_SAFE_SPEED = 0.50           # fracción de velocidad del movimiento automático a reposo

# --- Herramienta frente a un tornillo ---------------------------------------
# Para el desplazamiento lateral: si el TCP está delante de un tornillo (a
# SCREW_STANDOFF de la pared, dentro de estas tolerancias), la columna de
# referencia es la del TCP (el desplazamiento lo lleva al mismo tornillo de la
# columna destino); si no, la del brazo (su base, link_0).
TCP_AT_SCREW_TOL = 0.03         # m en lateral, altura y distancia a la pared
TCP_AT_SCREW_ANGLE = 5.0        # ° de inclinación del eje de la herramienta

# --- Check del TCP (check_tcp_orientation) ---------------------------------
# Tras llegar delante de un tornillo se compara por TF el TCP real con el
# pedido. Fuera de tolerancia -> aviso en el dashboard (y, con Servo en el 5.4,
# corrección corta).
TCP_CHECK_POS_TOL = 0.002       # m
TCP_CHECK_ANGLE_TOL = 1.0       # ° entre el eje de la herramienta real y el pedido
TCP_CHECK_DELAY = 0.3           # s de espera tras terminar el movimiento (TF al día)
# Corrección con Servo (5.4): por encima de esto no se corrige, se para y se avisa
TCP_CORRECTION_MAX_POS = 0.01   # m
TCP_CORRECTION_MAX_ANGLE = 3.0  # °
TCP_CORRECTION_ATTEMPTS = 3

# --- Desplazamiento lateral entre tornillos (ShiftAlongWall, secuenciador) ---
# El gateway solo usa SHIFT_MAX_DISTANCE (para rechazar antes de enviar); el
# resto los aplica el nodo wall_approach (wall_approach.yaml) y se dejan aquí
# como referencia de lo acordado.
SHIFT_MAX_SPEED = 0.3           # m/s
SHIFT_ACCEL = 0.5               # m/s²
SHIFT_MAX_DISTANCE = 1.0        # m: hasta aquí, con el brazo a SCREW_STANDOFF y sin postura segura
SHIFT_TOL_DISTANCE = 0.005      # m  \
SHIFT_TOL_ANGLE = 0.5           # °   > comprobación final tras el desplazamiento
SHIFT_TOL_LATERAL = 0.01        # m  /

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
