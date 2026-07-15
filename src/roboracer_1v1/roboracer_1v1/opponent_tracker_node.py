#!/usr/bin/env python3
"""
opponent_tracker_node.py
------------------------
LiDAR 클러스터링(DBSCAN)으로 상대 차량 위치를 추정한다.
- /scan → 클러스터링 → 상대 차량 후보 선택
- 발행: /opponent/pose (geometry_msgs/PoseStamped)
         /opponent/marker (visualization_msgs/Marker)
         /opponent/detected (std_msgs/Bool)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker

try:
    from sklearn.cluster import DBSCAN
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False


class OpponentTrackerNode(Node):
    def __init__(self):
        super().__init__('opponent_tracker_node')

        self.declare_parameter('detection_range_m',       8.0)
        self.declare_parameter('cluster_eps',             0.3)
        self.declare_parameter('cluster_min_samples',     3)
        self.declare_parameter('vehicle_width_min',       0.1)
        self.declare_parameter('vehicle_width_max',       0.6)
        self.declare_parameter('vehicle_height_min',      0.05)
        self.declare_parameter('lost_timeout_sec',        0.5)
        self.declare_parameter('smoothing_alpha',         0.4)

        self.det_range   = self.get_parameter('detection_range_m').value
        self.eps         = self.get_parameter('cluster_eps').value
        self.min_samples = self.get_parameter('cluster_min_samples').value
        self.w_min       = self.get_parameter('vehicle_width_min').value
        self.w_max       = self.get_parameter('vehicle_width_max').value
        self.h_min       = self.get_parameter('vehicle_height_min').value
        self.timeout     = self.get_parameter('lost_timeout_sec').value
        self.alpha       = self.get_parameter('smoothing_alpha').value

        self.smoothed_x   = None
        self.smoothed_y   = None
        self.last_seen    = self.get_clock().now()
        self.detected     = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)

        self.pub_pose     = self.create_publisher(PoseStamped, '/opponent/pose',     10)
        self.pub_detected = self.create_publisher(Bool,        '/opponent/detected', 10)
        self.pub_dist     = self.create_publisher(Float32,     '/opponent/distance', 10)
        self.pub_marker   = self.create_publisher(Marker,      '/opponent/marker',   10)

        self.timer = self.create_timer(0.1, self.check_timeout)

        if not SKLEARN_AVAILABLE:
            self.get_logger().warn('scikit-learn not found. Using simple nearest-cluster fallback.')

        self.get_logger().info('OpponentTrackerNode ready.')

    def scan_cb(self, msg: LaserScan):
        pts = self._scan_to_xy(msg)
        if len(pts) == 0:
            return

        clusters = self._cluster(pts)
        best = self._select_opponent(clusters)

        if best is None:
            return

        cx, cy = best
        now = self.get_clock().now()

        if self.smoothed_x is None:
            self.smoothed_x, self.smoothed_y = cx, cy
        else:
            self.smoothed_x = self.alpha * cx + (1 - self.alpha) * self.smoothed_x
            self.smoothed_y = self.alpha * cy + (1 - self.alpha) * self.smoothed_y

        self.last_seen = now
        self.detected  = True

        self._publish(msg.header.frame_id or 'laser', now)

    def _scan_to_xy(self, msg: LaserScan):
        angles = np.arange(msg.angle_min,
                           msg.angle_min + msg.angle_increment * len(msg.ranges),
                           msg.angle_increment)[:len(msg.ranges)]
        ranges = np.array(msg.ranges, dtype=np.float32)

        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < min(msg.range_max, self.det_range))
        angles = angles[valid]
        ranges = ranges[valid]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return np.column_stack([x, y])

    def _cluster(self, pts):
        if len(pts) < self.min_samples:
            return []

        if SKLEARN_AVAILABLE:
            labels = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit_predict(pts)
            clusters = []
            for lbl in set(labels):
                if lbl == -1:
                    continue
                clusters.append(pts[labels == lbl])
        else:
            clusters = [pts]

        return clusters

    def _select_opponent(self, clusters):
        best_dist = np.inf
        best_center = None

        for pts in clusters:
            w = pts[:, 0].max() - pts[:, 0].min()
            h = pts[:, 1].max() - pts[:, 1].min()
            span = max(w, h)

            if not (self.w_min <= span <= self.w_max):
                continue
            if min(w, h) < self.h_min:
                continue

            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            dist = np.hypot(cx, cy)

            if dist < best_dist:
                best_dist = dist
                best_center = (cx, cy)

        return best_center

    def _publish(self, frame_id, stamp):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp    = stamp.to_msg()
        pose.pose.position.x = self.smoothed_x
        pose.pose.position.y = self.smoothed_y
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

        det = Bool(); det.data = True
        self.pub_detected.publish(det)

        d = Float32()
        d.data = float(np.hypot(self.smoothed_x, self.smoothed_y))
        self.pub_dist.publish(d)

        m = Marker()
        m.header = pose.header
        m.type   = Marker.CUBE
        m.action = Marker.ADD
        m.pose   = pose.pose
        m.scale.x = 0.4; m.scale.y = 0.3; m.scale.z = 0.15
        m.color.r = 1.0; m.color.g = 0.3; m.color.b = 0.0; m.color.a = 0.9
        self.pub_marker.publish(m)

    def check_timeout(self):
        if not self.detected:
            return
        elapsed = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.detected     = False
            self.smoothed_x   = None
            self.smoothed_y   = None
            det = Bool(); det.data = False
            self.pub_detected.publish(det)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()