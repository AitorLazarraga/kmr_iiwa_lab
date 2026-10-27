#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,  TransformListener, TransformException
from kuka_interfaces.action import ApproachWall
from rclpy.qos import qos_profile_sensor_data
import rclpy.time
import numpy as np

import math

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class WallApproach(Node):
    def __init__(self):
        super().__init__('wall_approach')
        self.action_name = self.declare_parameter('action_name', '/kmr_iiwa/approach_wall').value
        self.scan_topic = self.declare_parameter('scan_topic', '/kmr_iiwa/scan').value
        self.vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel_smoothed').value
        self.map = self.declare_parameter('map_frame', 'map').value
        self.base = self.declare_parameter('base_frame', 'kmp_base_link').value
        self.rate = self.declare_parameter('rate', 20.0).value
        self.tol_distance = self.declare_parameter('tol_distance', 0.01).value
        self.tol_lateral = self.declare_parameter('tol_lateral', 0.015).value
        self.tol_angle = self.declare_parameter('tol_angle', 0.0175).value
        self.max_vx = self.declare_parameter('max_vx', 0.08).value
        self.max_vy = self.declare_parameter('max_vy', 0.06).value
        self.max_wz = self.declare_parameter('max_wz', 0.15).value
        self.cycles = self.declare_parameter('stable_cycles', 10).value
        self.timeout = self.declare_parameter('timeout', 30.0).value
        self.min_wall_points = self.declare_parameter('min_wall_points', 20).value
        self.base_half = self.declare_parameter('base_half', 0.54).value
        self.arm_offset = self.declare_parameter('arm_offset', -0.1842).value
        self.window_band = self.declare_parameter('window_band', 0.25).value
        self.inlier_tol = self.declare_parameter('inlier_tolerance', 0.02).value
        self.ransac_iters = self.declare_parameter('ransac_iters', 60).value
        self.lost_cycles = self.declare_parameter('lost_cycles', 10).value
        self.k_dist = self.declare_parameter('k_dist', 0.8).value
        self.k_lat = self.declare_parameter('k_lat', 0.8).value
        self.k_ang = self.declare_parameter('k_ang', 1.0).value
        self.max_reverse = self.declare_parameter('max_reverse', 0.03).value
        self.max_correction = self.declare_parameter('max_correction', 0.4).value

        self.busy = False

        self.scan = None

        self.cb = ReentrantCallbackGroup()

        self.server = ActionServer(self, ApproachWall, self.action_name,
                           execute_callback=self.execute,
                           goal_callback=self.on_goal,
                           cancel_callback=self.on_cancel,
                           callback_group=self.cb)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(TwistStamped, self.vel_topic, 10)

        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.recibir_scan,qos_profile_sensor_data, callback_group=self.cb)

    def recibir_scan(self, msg):
        self.scan = msg

    def result_from(self, ok, message, m):
        """Result con los errores de la última medida (o ceros si no hubo ninguna)."""
        if m is None:
            return self.make_result(ok, message)
        return self.make_result(ok, message, m['distance_error'], m['lateral_error'], m['angle_error'])

    def execute(self, goal_handle):
        self.busy = True
        g = goal_handle.request
        self.get_logger().info(
            f'Acercamiento: pared ({g.wall_p1.x:.2f}, {g.wall_p1.y:.2f}) -> '
            f'({g.wall_p2.x:.2f}, {g.wall_p2.y:.2f}), distancia {g.distance:.2f} m, lateral {g.lateral:.2f} m')
        rate = self.create_rate(self.rate)
        start = self.get_clock().now()
        feedback = ApproachWall.Feedback()
        lost = 0                     # ciclos seguidos sin pared
        stable = 0
        last = None                  # última medida buena (para el Result)
        try:
            while rclpy.ok():
                # 1. cancelación
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self.result_from(False, 'cancelado', last)
                # 2. tiempo
                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
                if elapsed > self.timeout:
                    goal_handle.abort()
                    return self.result_from(False, f'prueba de medida terminada ({self.timeout:.0f} s)', last)
                # 3-4. medir; tolerar unos ciclos sin pared
                m = self.measure(g)
                if m is None:
                    lost += 1
                    if lost > self.lost_cycles:
                        goal_handle.abort()
                        return self.result_from(False, 'pared perdida: no se ve en la zona esperada', last)
                    rate.sleep()
                    continue
                lost = 0
                
                last = m
                            # 5a. ¿demasiado lejos para un ajuste fino?
                if max(abs(m['distance_error']), abs(m['lateral_error'])) > self.max_correction:
                    goal_handle.abort()
                    return self.result_from(False, 'demasiado lejos para el ajuste fino: '
                                                'vuelve a navegar al punto', m)
                # 5b. control
                vx, vy, wz, in_tol = self.control(m)
                self.publish_cmd(vx, vy, wz)          # dentro de tolerancia sigue afinando (zona muerta)
                
                stable = stable + 1 if in_tol else 0
                # 6. feedback
                feedback.phase = 'estable' if in_tol else 'corregir'
                feedback.distance_error = m['distance_error']
                feedback.lateral_error = m['lateral_error']
                feedback.angle_error = m['angle_error']
                feedback.wall_points = m['points']
                goal_handle.publish_feedback(feedback)
                # 7. ¿terminado?
                if stable >= self.cycles:
                    goal_handle.succeed()
                    return self.result_from(True, 'posicionado frente a la pared', m)
                # 8. esperar al siguiente ciclo
                rate.sleep()
        finally:
            self.busy = False
            self.stop()
            self.destroy_rate(rate)
            

    def control(self, m):
        de, le, ae = m['distance_error'], m['lateral_error'], m['angle_error']
        v = np.zeros(2)
        if abs(de) > self.tol_distance / 2:
            v += self.k_dist * de * (-m['n'])
        if abs(le) > self.tol_lateral / 2:
            v += -self.k_lat * le * m['w']
        wz = self.k_ang * ae if abs(ae) > self.tol_angle / 2 else 0.0

        vx,vy = float(v[0]), float(v[1])
        k = 1.0
        if vx > self.max_vx:
            k = min(k, self.max_vx / vx)
        if vx < -self.max_reverse:
            k = min(k, self.max_reverse / -vx)
        if abs(vy) > self.max_vy:
            k = min(k, self.max_vy / abs(vy))
        vx, vy = vx * k, vy * k
        wz = max(-self.max_wz, min(self.max_wz, wz))

        in_tol = (abs(de) <= self.tol_distance and abs(le) <= self.tol_lateral and abs(ae)<= self.tol_angle)

        return vx, vy, wz, in_tol
    
    def robot_pose(self):
        """(x, y, yaw) de la base en el mapa según la localización, o None."""
        try:
            tf = self.tf_buffer.lookup_transform(self.map, self.base, rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    def scan_points(self):
        scan = self.scan
        if scan is None:
            return None
        r = np.asarray(scan.ranges, dtype=float)                     
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment   
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        r, a = r[ok], a[ok]                           
        pts = np.column_stack((r * np.cos(a), r * np.sin(a)))        

        if scan.header.frame_id != self.base:               
            try:
                tf = self.tf_buffer.lookup_transform(self.base, scan.header.frame_id,
                                                    rclpy.time.Time())
            except TransformException:
                return None
            t = tf.transform.translation
            yaw = yaw_from_quat(tf.transform.rotation)
            c, s = math.cos(yaw), math.sin(yaw)
            rot = np.array([[c, -s], [s, c]])
            pts = pts @ rot.T + np.array([t.x, t.y])              
        return pts

    def fit_line(self, pts):
        n = len(pts)
        if n < 2:
            return None
        rng = np.random.default_rng()
        best_mask, best_count = None, 0
        for _ in range(self.ransac_iters):
            i, j = rng.choice(n, 2, replace=False)        # 2 puntos distintos al azar
            d = pts[j] - pts[i]
            length = np.hypot(d[0], d[1])
            if length < 1e-3:                             # casi el mismo punto: no define recta
                continue
            d = d / length
            normal = np.array([-d[1], d[0]])
            dist = np.abs((pts - pts[i]) @ normal)        # distancia de TODOS los puntos a la recta
            mask = dist < self.inlier_tol                 # True = inlier
            count = int(mask.sum())
            if count > best_count:
                best_mask, best_count = mask, count
        if best_mask is None or best_count < 2:
            return None
        inl = pts[best_mask]
        center = inl.mean(axis=0)                         # centroide
        _, _, vt = np.linalg.svd(inl - center)            # dirección principal = vt[0]
        return center, vt[0], best_count

    


    def measure(self, goal):
        pose = self.robot_pose()
        pts = self.scan_points()
        if pose is None or pts is None or len(pts) < 2:
            return None
        rx, ry, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)

        # 1. Dónde DEBERÍA estar la pared, en el frame de la base: p_base = R(-yaw)·(p - robot)
        def to_base(p):
            dx, dy = p.x - rx, p.y - ry
            return np.array([c * dx + s * dy, -s * dx + c * dy])
        
        a, b = to_base(goal.wall_p1), to_base(goal.wall_p2)
        length = float(np.linalg.norm(b - a))
        t = (b - a) / length                      # a lo largo del segmento esperado
        nrm = np.array([-t[1], t[0]])             # perpendicular

        # 2. Ventana: solo los puntos cerca de ese segmento (ni otras paredes ni obstáculos)
        rel = pts - a
        along = rel @ t                           # posición a lo largo del segmento
        across = rel @ nrm                        # distancia perpendicular al segmento
        band = self.window_band
        window = (np.abs(across) < band) & (along > -band) & (along < length + band)
        candidates = pts[window]
        if len(candidates) < self.min_wall_points:
            return None

        # 3. Recta real con RANSAC
        fit = self.fit_line(candidates)
        if fit is None or fit[2] < self.min_wall_points:
            return None
        center, d, count = fit

        # 4. Normal de la recta medida orientada HACIA el robot (el robot está en el origen)
        n = np.array([-d[1], d[0]])
        if n @ center > 0:                        # apunta hacia la pared: darle la vuelta
            n = -n

        w = d if d @ t > 0 else -d

        # Distancia: del centro de la base a la recta, menos medio robot = frontal a la pared
        distance_error = (abs(float(n @ center)) - self.base_half) - goal.distance
        # Ángulo: dirección hacia la pared (-n) respecto al eje X del robot.
        # Positivo = la pared queda girada a la izquierda -> girar antihorario
        angle_error = math.atan2(-n[1], -n[0])

        # 5. Lateral con la localización (opción A), en el mapa
        p1 = np.array([goal.wall_p1.x, goal.wall_p1.y])
        p2 = np.array([goal.wall_p2.x, goal.wall_p2.y])
        tm = (p2 - p1) / np.linalg.norm(p2 - p1)                 # dirección p1 -> p2
        target = (p1 + p2) / 2 + tm * goal.lateral               # donde debe quedar el brazo
        arm = np.array([rx, ry]) + self.arm_offset * np.array([-s, c])   # brazo: arm_offset sobre el eje Y del robot
        lateral_error = float((arm - target) @ tm)               # + = brazo desplazado hacia p2

        return {'distance_error': distance_error, 'angle_error': angle_error,
                'lateral_error': lateral_error, 'points': count, 'n': n, 'w': w}    


    def publish_cmd(self, vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0, 0.0)

    def make_result(self, ok, message,d=0.0,l=0.0,a=0.0):
        res = ApproachWall.Result()
        res.success = ok
        res.message = message
        res.final_distance_error = d
        res.final_lateral_error = l
        res.final_angle_error = a
        return res

    def on_goal(self, goal):
        if self.busy:
            self.get_logger().warning('Meta rechazada: ya hay un acercamiento en curso')
            return GoalResponse.REJECT
        nums = (goal.wall_p1.x, goal.wall_p1.y, goal.wall_p2.x, goal.wall_p2.y, goal.distance, goal.lateral)
        if not all(math.isfinite(v) for v in nums):
            self.get_logger().warning('Meta rechazada: valores no finitos')
            return GoalResponse.REJECT
        tramo = math.hypot(goal.wall_p2.x - goal.wall_p1.x, goal.wall_p2.y - goal.wall_p1.y)
        if tramo < 0.3:
            self.get_logger().warning(f'Meta rechazada: tramo de pared de {tramo:.2f} m (mínimo 0.3)')
            return GoalResponse.REJECT
        if not 0.5 <= goal.distance <= 2.0:
            self.get_logger().warning(f'Meta rechazada: distancia {goal.distance:.2f} m fuera de [0.5, 2.0]')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

def main():
    rclpy.init()
    node = WallApproach()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()