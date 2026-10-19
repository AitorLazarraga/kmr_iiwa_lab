"""Fusiona varios LaserScan en uno solo de 360° expresado en el frame de la base.

Cada scan de entrada se transforma (TF) al frame destino, sus puntos se pasan
a coordenadas polares respecto al centro de la base y se agrupan por ángulo,
quedándose con la distancia mínima de cada sector.

Parámetros:
    scan_topics      lista de topics de entrada
    output_topic     topic de salida
    target_frame     frame del scan fusionado (centro de la base)
    angle_increment  resolución angular de salida [rad]
    range_min/max    rango válido de salida [m]
    sync_slop        diferencia máxima de tiempo entre scans para fusionarlos [s]
"""
import math

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ScanMerger(Node):

    def __init__(self):
        super().__init__('scan_merger')

        self.declare_parameter('scan_topics', [
            '/kmr_iiwa/lidar_front_right',
            '/kmr_iiwa/lidar_back_left',
        ])
        self.declare_parameter('output_topic', '/kmr_iiwa/scan')
        self.declare_parameter('target_frame', 'kmp_base_link')
        self.declare_parameter('angle_increment', 0.0058)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('sync_slop', 0.05)

        topics = self.get_parameter('scan_topics').value
        self.target = self.get_parameter('target_frame').value
        self.rmin = float(self.get_parameter('range_min').value)
        self.rmax = float(self.get_parameter('range_max').value)
        inc = float(self.get_parameter('angle_increment').value)
        self.n_bins = int(round(2 * math.pi / inc))
        self.inc = 2 * math.pi / self.n_bins

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_cache = {}  # frame_id -> (R 3x3, t 3)

        # Entrada best_effort: compatible con publicadores reliable y best_effort
        self.subs = [Subscriber(self, LaserScan, t, qos_profile=qos_profile_sensor_data)
                     for t in topics]
        self.sync = ApproximateTimeSynchronizer(
            self.subs, queue_size=10,
            slop=float(self.get_parameter('sync_slop').value))
        self.sync.registerCallback(self.callback)

        # Salida reliable: la aceptan suscriptores reliable (SLAM) y best_effort
        self.pub = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value, 10)

        self.get_logger().info(
            f'Fusionando {topics} -> {self.get_parameter("output_topic").value} '
            f'en {self.target} ({self.n_bins} haces)')

    def get_tf(self, frame_id):
        """Los lidars están fijos en la base: la TF se consulta una vez y se guarda."""
        if frame_id in self.tf_cache:
            return self.tf_cache[frame_id]
        try:
            tf = self.tf_buffer.lookup_transform(self.target, frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'Sin TF {self.target} <- {frame_id} todavía: {e}',
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        result = (quat_to_matrix(tf.transform.rotation), np.array([t.x, t.y, t.z]))
        self.tf_cache[frame_id] = result
        return result

    def callback(self, *scans):
        xs, ys = [], []
        for s in scans:
            tf = self.get_tf(s.header.frame_id)
            if tf is None:
                return
            R, t = tf
            r = np.asarray(s.ranges, dtype=np.float64)
            ang = s.angle_min + np.arange(r.size) * s.angle_increment
            valid = (np.isfinite(r)
                     & (r >= max(s.range_min, self.rmin))
                     & (r <= min(s.range_max, self.rmax)))
            r, ang = r[valid], ang[valid]
            pts = np.vstack((r * np.cos(ang), r * np.sin(ang), np.zeros_like(r)))
            p = R @ pts + t[:, None]
            xs.append(p[0])
            ys.append(p[1])

        ranges = np.full(self.n_bins, np.inf, dtype=np.float32)
        x, y = np.concatenate(xs), np.concatenate(ys)
        if x.size:
            rr = np.hypot(x, y)
            aa = np.arctan2(y, x)
            keep = (rr >= self.rmin) & (rr <= self.rmax)
            idx = np.floor((aa[keep] + math.pi) / self.inc).astype(int) % self.n_bins
            np.minimum.at(ranges, idx, rr[keep].astype(np.float32))

        out = LaserScan()
        newest = max(scans, key=lambda s: (s.header.stamp.sec, s.header.stamp.nanosec))
        out.header.stamp = newest.header.stamp
        out.header.frame_id = self.target
        out.angle_min = -math.pi
        out.angle_increment = self.inc
        out.angle_max = -math.pi + (self.n_bins - 1) * self.inc
        out.time_increment = 0.0
        out.scan_time = scans[0].scan_time
        out.range_min = self.rmin
        out.range_max = self.rmax
        out.ranges = ranges.tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
