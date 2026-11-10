"""Cliente de MoveIt para el gateway (paso 5.3).

No es un nodo propio: usa el nodo del gateway (RosBridge) para crear el cliente
de la acción MoveGroup y el del servicio de la escena de planificación.

  - Movimientos planificados con comprobación de colisiones (plan + ejecución
    en una sola llamada a /kmr_iiwa/move_action):
      * a una postura articular (7 ángulos)
      * a una pose del TCP (posición + orientación completa de la herramienta)
  - La pared del punto de trabajo como caja en la escena de colisiones, para
    que ningún plan la atraviese.
  - Geometría de "ir a tornillo": pose del TCP delante de un tornillo de la
    rejilla, con la herramienta perpendicular a la pared. Se calcula en el frame
    de la BASE (no con la línea marcada en el mapa): tras el ajuste la base está
    perpendicular a la pared real, así que la herramienta también.
  - check_tcp_orientation (la parte de cálculo, tcp_errors): cuánto se separa
    el TCP real del pedido.

Cada objetivo se calcula en el frame que convenga (hoy la base; con el robot
real, la cámara que detecte el tornillo) y se pasa a la base del brazo
(ARM_BASE_FRAME, kmr_iiwa_link_0) con TF antes de enviarlo a MoveIt.
"""
import logging
import math

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint, PlanningScene,
                             PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformException

from .errors import CommandRejected

log = logging.getLogger('gateway.planner')

WALL_ID = 'pared_trabajo'

# Códigos de MoveIt -> texto para el dashboard
ERROR_TEXT = {
    MoveItErrorCodes.PLANNING_FAILED: 'no se encontró una trayectoria sin colisiones',
    MoveItErrorCodes.INVALID_MOTION_PLAN: 'el plan dejó de ser válido',
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: 'el entorno cambió durante el movimiento',
    MoveItErrorCodes.CONTROL_FAILED: 'el controlador no pudo ejecutar la trayectoria',
    MoveItErrorCodes.TIMED_OUT: 'tiempo agotado',
    MoveItErrorCodes.PREEMPTED: 'cancelado',
    MoveItErrorCodes.START_STATE_IN_COLLISION: 'el brazo ya está en colisión en la posición actual',
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: 'la posición actual no cumple las restricciones',
    MoveItErrorCodes.GOAL_IN_COLLISION: 'el objetivo está en colisión',
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: 'el objetivo no cumple las restricciones',
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'el objetivo no es alcanzable con esas restricciones',
    MoveItErrorCodes.INVALID_GROUP_NAME: 'grupo de planificación desconocido',
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'objetivo mal definido',
    MoveItErrorCodes.INVALID_ROBOT_STATE: 'estado del robot no válido',
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: 'fallo de transformación de frames',
    MoveItErrorCodes.NO_IK_SOLUTION: 'sin solución de cinemática inversa (fuera de alcance)',
}


def error_text(code):
    return ERROR_TEXT.get(code, f'error de MoveIt {code}')


# ---------------------------------------------------------------- geometría
def quat_mul(a, b):
    """Producto de cuaterniones (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    """Rota el vector v por el cuaternión q."""
    x, y, z, w = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def quat_from_axes(x_axis, y_axis, z_axis):
    """Cuaternión de una matriz de rotación dada por sus columnas (ejes unitarios)."""
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


def wall_frame(wall):
    """Centro del tramo, dirección a lo largo (p1 -> p2), dirección hacia la pared
    y longitud (la misma convención que work.py)."""
    (x1, y1), (x2, y2) = wall
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    t = (dx / length, dy / length)
    f = (-t[1], t[0])
    return ((x1 + x2) / 2, (y1 + y2) / 2), t, f, length


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


# Herramienta mirando al +X de la base (hacia la pared tras el ajuste). Ejes de
# la pose en el frame de la base: Z herramienta = +X base; X herramienta = +Z
# base (hacia arriba). La orientación se pide completa (ver pose_goal), así que
# este giro sobre el eje de la herramienta es el que tendrá: es el que salió de
# forma natural en las pruebas de Gazebo, con el brazo cómodo.
TOOL_TO_WALL_QUAT = quat_from_axes((0.0, 0.0, 1.0), cross((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
                                   (1.0, 0.0, 0.0))


def wall_to_base(pos, angle):
    """Del frame «pared» al de la base. El frame pared tiene el origen en el
    centro de la base, X perpendicular a la pared REAL (hacia ella) e Y a lo
    largo; está girado `angle` rad alrededor de Z respecto a la base (el error
    angular que deja el ajuste: + = la pared queda girada a la izquierda)."""
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = pos
    return (c * x - s * y, s * x + c * y, z)


def screw_tcp_pose_base(distance, screw, standoff, base_half, arm_offset_y, base_height=0.0,
                        lateral=0.0, angle=0.0):
    """Pose del TCP delante de un tornillo, en el frame de la BASE (kmp_base_link).

    Parte de la posición REAL de la base frente a la pared (la medida por el
    ajuste o el desplazamiento lateral, no la pedida):
      - distance: del frontal a la cara de la pared -> la cara está en
        x = base_half + distance (frame pared)
      - lateral: dónde está el brazo a lo largo del tramo, con el mismo
        convenio que `u` (+ hacia el 2º punto = derecha vista desde el robot).
        El tornillo `u` queda (u - lateral) a la derecha del brazo:
        y = arm_offset_y - (u - lateral)
      - angle: error angular que queda; el objetivo se gira lo mismo para que
        la herramienta siga perpendicular a la pared real
      - base_height: altura del origen de la base sobre el suelo (las alturas
        de los tornillos son sobre el suelo)
    La herramienta apunta al +X del frame pared: perpendicular a la pared
    real, sin depender de lo bien marcada que esté la línea en el mapa."""
    pos_wall = (base_half + distance - standoff,
                arm_offset_y - (screw['u'] - lateral),
                screw['z'] - base_height)
    rot = (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))
    return wall_to_base(pos_wall, angle), quat_mul(rot, TOOL_TO_WALL_QUAT)


def tcp_errors(actual_pos, actual_quat, target_pos, target_quat):
    """check_tcp_orientation, la parte de cálculo: error de posición (m) y ángulo
    (°) entre el eje Z de la herramienta real y el pedido. El giro sobre el
    propio eje no cuenta: lo que importa es la perpendicularidad."""
    pos_err = math.dist(actual_pos, target_pos)
    za = quat_rotate(actual_quat, (0.0, 0.0, 1.0))
    zt = quat_rotate(target_quat, (0.0, 0.0, 1.0))
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(za, zt))))
    return pos_err, math.degrees(math.acos(dot))


class MoveItPlanner:
    def __init__(self, node, cfg):
        self.node = node
        self.cfg = cfg
        self.client = ActionClient(node, MoveGroup, cfg.MOVEIT_ACTION)
        self.scene_cli = node.create_client(ApplyPlanningScene, cfg.MOVEIT_SCENE_SERVICE)
        self.wall = None                  # nombre del punto cuya pared está en la escena
        log.info('MoveIt: acción %s, escena %s, grupo %s', cfg.MOVEIT_ACTION,
                 cfg.MOVEIT_SCENE_SERVICE, cfg.MOVEIT_GROUP)

    def ready(self):
        return self.client.server_is_ready()

    def check_ready(self):
        if not self.ready():
            raise CommandRejected('MoveIt no está en marcha (move_group): lanza '
                                  'kuka_moveit_config/moveit_sim.launch.py para mover el brazo '
                                  'con planificación')

    # ------------------------------------------------------------ TF
    def frame_to_arm_base(self, frame):
        """(traslación, rotación) de `frame` expresado en la base del brazo
        (ARM_BASE_FRAME), o CommandRejected si TF no la tiene. Cualquier objetivo
        (en el mapa, en la base, en la cámara…) pasa por aquí antes de ir a MoveIt."""
        try:
            tf = self.node.tf_buffer.lookup_transform(self.cfg.ARM_BASE_FRAME, frame, Time())
        except TransformException as e:
            raise CommandRejected(f'sin transformada {frame} -> {self.cfg.ARM_BASE_FRAME} ({e})')
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def map_to_arm_base(self):
        return self.frame_to_arm_base(self.cfg.MAP_FRAME)

    def to_arm_base(self, frame, pos, quat=None):
        """Pose en `frame` -> pose en ARM_BASE_FRAME."""
        return self.transform(self.frame_to_arm_base(frame), pos, quat)

    def tcp_in_base(self):
        """(pos, quat) del TCP en el frame de la base, o None si TF no lo tiene."""
        try:
            tf = self.node.tf_buffer.lookup_transform(self.cfg.BASE_FRAME, self.cfg.ARM_TCP_FRAME, Time())
        except TransformException:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def base_height(self):
        """Altura del origen de la base (BASE_FRAME) sobre el suelo (z del mapa).
        Las alturas de los tornillos son sobre el suelo."""
        try:
            tf = self.node.tf_buffer.lookup_transform(self.cfg.MAP_FRAME, self.cfg.BASE_FRAME, Time())
        except TransformException as e:
            raise CommandRejected(f'sin transformada {self.cfg.BASE_FRAME} -> {self.cfg.MAP_FRAME} ({e})')
        return tf.transform.translation.z

    @staticmethod
    def transform(tf, pos, quat=None):
        trans, rot = tf
        p = quat_rotate(rot, pos)
        p = (p[0] + trans[0], p[1] + trans[1], p[2] + trans[2])
        return p, (quat_mul(rot, quat) if quat is not None else None)

    # ------------------------------------------------------------ objetivos
    def _base_goal(self, scaling, lin=False):
        """lin=True: movimiento LIN de Pilz (línea recta del TCP); si no, OMPL."""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.cfg.MOVEIT_GROUP
        req.num_planning_attempts = 3
        req.allowed_planning_time = float(self.cfg.MOVEIT_PLANNING_TIME)
        req.max_velocity_scaling_factor = float(scaling)
        req.max_acceleration_scaling_factor = float(scaling)
        if lin:
            req.pipeline_id = self.cfg.PILZ_PIPELINE
            req.planner_id = 'LIN'
            req.num_planning_attempts = 1         # Pilz es determinista
        else:
            req.pipeline_id = 'ompl'
            req.planner_id = ''                   # el planificador por defecto del grupo
        req.start_state.is_diff = True            # desde el estado actual
        goal.planning_options.plan_only = False   # planificar y ejecutar
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.replan = False
        return goal

    def joint_goal(self, joints, positions, scaling):
        goal = self._base_goal(scaling)
        c = Constraints()
        for name, p in zip(joints, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(p)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints = [c]
        return goal

    def pose_goal(self, pos, quat, scaling, position_tol=None, axis_tol=None, lin=False):
        """Pose del TCP en ARM_BASE_FRAME, con la orientación COMPLETA fijada
        (±axis_tol en los tres ejes).

        Antes se dejaba ±π en Z para que el giro de la herramienta sobre su eje
        fuera libre. En las pruebas en Gazebo se vio que MoveIt aplica esa
        tolerancia sobre el eje Z del frame de referencia (link_0, vertical) y
        no sobre el eje de la herramienta: el giro sobre su eje quedaba fijo y,
        en cambio, la herramienta podía girar libremente en horizontal, de ahí
        los errores de 12–50° respecto a la normal de la pared y los
        movimientos largos y raros (cada plan elegía una orientación al azar).
        Ahora el giro lo elegimos nosotros (en TOOL_TO_WALL_QUAT, eje X de la
        herramienta hacia arriba) y MoveIt solo tiene que alcanzarlo.

        lin=True: Pilz LIN, el TCP va en línea recta desde donde está hasta la
        pose (Pilz usa el centro de la restricción como objetivo exacto).
        `scaling` es entonces la fracción de PILZ_MAX_TRANS_VEL."""
        position_tol = self.cfg.MOVEIT_POSITION_TOL if position_tol is None else position_tol
        axis_tol = self.cfg.MOVEIT_AXIS_TOL if axis_tol is None else axis_tol
        goal = self._base_goal(scaling, lin=lin)
        frame, link = self.cfg.ARM_BASE_FRAME, self.cfg.ARM_TCP_FRAME
        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(position_tol)]
        center = Pose()
        center.position.x, center.position.y, center.position.z = (float(v) for v in pos)
        center.orientation.w = 1.0
        pc.constraint_region = BoundingVolume()
        pc.constraint_region.primitives = [sphere]
        pc.constraint_region.primitive_poses = [center]
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = link
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = (float(v) for v in quat)
        oc.absolute_x_axis_tolerance = float(axis_tol)
        oc.absolute_y_axis_tolerance = float(axis_tol)
        oc.absolute_z_axis_tolerance = float(axis_tol)   # orientación completa: ver docstring
        oc.parameterization = OrientationConstraint.ROTATION_VECTOR
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        goal.request.goal_constraints = [c]
        return goal

    def send(self, goal, feedback_cb):
        return self.client.send_goal_async(goal, feedback_callback=feedback_cb)

    # ------------------------------------------------------------ escena
    def set_wall(self, name, work, est):
        """Añade (o sustituye) la pared del punto de trabajo como una caja. Se
        calcula en el frame de la base con la misma posición real que los
        tornillos (`est` = {distance, lateral, angle}, ver screw_tcp_pose_base):
        la cara a `distance` del frontal, el centro del tramo `lateral` a la
        izquierda del brazo, girada `angle`; WORK_WALL_THICKNESS de grosor y
        WORK_WALL_HEIGHT de alto. Tras mover la base hay que volver a llamarla:
        la caja se guarda respecto a la base y no se mueve sola con el mapa."""
        if not self.scene_cli.service_is_ready():
            log.warning('MoveIt: sin servicio de escena; la pared de %s no se añade', name)
            return False
        cfg = self.cfg
        _, _, _, length = wall_frame(work['wall'])
        thick, height = cfg.WORK_WALL_THICKNESS, cfg.WORK_WALL_HEIGHT
        a = est['angle']
        try:
            center_wall = (cfg.BASE_HALF_LENGTH + est['distance'] + thick / 2,
                           cfg.ARM_OFFSET_Y + est['lateral'],
                           height / 2 - self.base_height())
            pos, quat = self.to_arm_base(cfg.BASE_FRAME, wall_to_base(center_wall, a),
                                         (0.0, 0.0, math.sin(a / 2), math.cos(a / 2)))
        except CommandRejected as e:
            log.warning('MoveIt: pared de %s no añadida: %s', name, e)
            return False

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [thick, length + 0.2, height]     # 10 cm de margen a cada lado
        obj = CollisionObject()
        obj.header.frame_id = cfg.ARM_BASE_FRAME
        obj.id = WALL_ID
        obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = pos
        obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w = quat
        identity = Pose()
        identity.orientation.w = 1.0
        obj.primitives = [box]
        obj.primitive_poses = [identity]
        obj.operation = CollisionObject.ADD
        self._apply(obj, f'pared de {name} añadida a la escena')
        self.wall = name
        return True

    def clear_wall(self):
        if self.wall is None:
            return
        name, self.wall = self.wall, None
        if not self.scene_cli.service_is_ready():
            return
        obj = CollisionObject()
        obj.header.frame_id = self.cfg.ARM_BASE_FRAME
        obj.id = WALL_ID
        obj.operation = CollisionObject.REMOVE
        self._apply(obj, f'pared de {name} retirada de la escena')

    def _apply(self, obj, what):
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        req.scene.world.collision_objects = [obj]
        future = self.scene_cli.call_async(req)

        def done(f):
            try:
                ok = f.result().success
            except Exception as e:  # noqa: BLE001
                ok = False
                log.warning('MoveIt: error al actualizar la escena (%s)', e)
            if ok:
                log.info('MoveIt: %s', what)
            else:
                log.warning('MoveIt: no se pudo actualizar la escena (%s)', what)
        future.add_done_callback(done)
