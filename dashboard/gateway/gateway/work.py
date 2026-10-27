"""Geometría de los puntos de trabajo (paso 6.3).

Funciones puras (sin ROS ni FastAPI) para poder probarlas sueltas.

Convención para marcar la pared: mirando la pared desde donde se colocará el
robot, primer punto a la IZQUIERDA y segundo a la DERECHA. El orden decide en
qué lado de la pared queda el robot (los paneles de prueba son exentos y tienen
espacio libre por los dos lados, así que no se puede deducir del mapa).

    wall     [[x1, y1], [x2, y2]]  cara de la pared, en el frame del mapa (m)
    distance del FRONTAL del robot a la cara de la pared (m)
    grid     {cols, rows, spacing: [h, v], bottom}  rejilla de tornillos
"""
import numpy as np

from . import config as cfg

MIN_WALL_LENGTH = 0.3          # m


def _wall_frame(wall):
    """Centro del tramo, dirección a lo largo (izq -> dcha), dirección hacia la
    pared (a donde mira el robot) y longitud."""
    p1, p2 = np.asarray(wall, dtype=float)
    along = p2 - p1
    length = float(np.linalg.norm(along))
    if length < 1e-6:
        raise ValueError('los dos puntos de la pared coinciden')
    t = along / length
    f = np.array([-t[1], t[0]])          # t girado +90°: de la derecha a "hacia delante"
    return (p1 + p2) / 2, t, f, length


def approach_pose(wall, distance):
    """Pose (x, y, yaw) donde Nav2 debe dejar el robot: mirando perpendicular a
    la pared, con el frontal a `distance` y el BRAZO (no el centro de la base)
    frente al centro del tramo marcado."""
    center, t, f, _ = _wall_frame(wall)
    pos = (center
           - f * (distance + cfg.BASE_HALF_LENGTH)   # retroceder: distancia + medio robot
           + t * cfg.ARM_OFFSET_Y)                    # el brazo está a la derecha del centro
    yaw = float(np.arctan2(f[1], f[0]))
    return float(pos[0]), float(pos[1]), yaw


def screws(wall, grid):
    """Tornillos de la rejilla: [{row, col, u, z, x, y}].

    u: m a lo largo de la pared desde el centro del tramo (+ hacia el segundo
    punto); z: altura; x, y: posición sobre la cara de la pared en el mapa.
    Fila 0 = abajo; columna 0 = lado del primer punto (izquierda vista desde el robot).
    """
    center, t, _, _ = _wall_frame(wall)
    cols, rows = int(grid['cols']), int(grid['rows'])
    sh, sv = grid['spacing']
    out = []
    for row in range(rows):
        for col in range(cols):
            u = (col - (cols - 1) / 2) * sh
            p = center + t * u
            out.append({
                'row': row, 'col': col,
                'u': round(float(u), 4),
                'z': round(float(grid['bottom'] + row * sv), 4),
                'x': round(float(p[0]), 4), 'y': round(float(p[1]), 4),
            })
    return out


def check(wall, distance, grid):
    """Comprobaciones que dependen de varios valores a la vez. Lanza ValueError
    con un mensaje legible. (Los rangos de cada campo los valida Pydantic.)"""
    pts = np.array(wall, dtype=float)
    if pts.shape != (2, 2) or not np.isfinite(pts).all():
        raise ValueError('la pared debe ser dos puntos [x, y] con valores finitos')
    values = [distance, grid['bottom'], *grid['spacing']]
    if not np.isfinite(np.array(values, dtype=float)).all():
        raise ValueError('distancia, altura y separaciones deben ser números finitos')

    length = float(np.linalg.norm(pts[1] - pts[0]))
    if length < MIN_WALL_LENGTH:
        raise ValueError(f'el tramo de pared marcado mide {length:.2f} m '
                         f'(mínimo {MIN_WALL_LENGTH} m)')
    width = (grid['cols'] - 1) * grid['spacing'][0]
    if width > length + 1e-9:
        raise ValueError(f'la rejilla mide {width:.2f} m de ancho y el tramo marcado '
                         f'solo {length:.2f} m: marca más pared o reduce columnas/separación')
    top = grid['bottom'] + (grid['rows'] - 1) * grid['spacing'][1]
    if top > cfg.WORK_MAX_HEIGHT + 1e-9:
        raise ValueError(f'la fila superior queda a {top:.2f} m '
                         f'(máximo alcanzable {cfg.WORK_MAX_HEIGHT:.2f} m)')


def geometry(work):
    """Todo lo calculado de un punto de trabajo, para la API y la web."""
    x, y, yaw = approach_pose(work['wall'], work['distance'])
    _, _, _, length = _wall_frame(work['wall'])
    return {'x': x, 'y': y, 'yaw': yaw, 'wall_length': round(length, 4),
            'screws': screws(work['wall'], work['grid'])}
