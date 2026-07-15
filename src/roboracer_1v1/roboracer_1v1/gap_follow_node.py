#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32
from ackermann_msgs.msg import AckermannDriveStamped


class GapFollowNode(Node):
    def __init__(self):
        super().__init__('gap_follow_node')

        self.declare_parameter('car_width',               0.35)
        self.declare_parameter('disparity_threshold',     0.3)
        self.declare_parameter('bubble_radius',           0.25)
        self.declare_parameter('speed_with_obstacles',    2.5)
        self.declare_parameter('speed_no_obstacles',      4.5)
        self.declare_parameter('speed_max',               5.5)
        self.declare_parameter('speed_min',               1.0)
        self.declare_parameter('steer_gain',              0.35)
        self.declare_parameter('steer_max_deg',           25.0)
        self.declare_parameter('opponent_virtual_dist',   1.5)
        self.declare_parameter('opponent_influence_deg',  12.0)
        self.declare_parameter('safety_distance',         0.8)
        self.declare_parameter('obstacle_removal_lap',    10)

        self._load_params()

        self.opponent_pose = None
        self.opponent_detected = False
        self.lap_count = 0
        self.overtaking_active = False
        self.enabled = True

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan     = self.create_subscription(LaserScan,   '/scan',              self.scan_cb,     scan_qos)
        self.sub_opp_pose = self.create_subscription(PoseStamped, '/opponent/pose',     self.opp_pose_cb, 10)
        self.sub_opp_det  = self.create_subscription(Bool,        '/opponent/detected', self.opp_det_cb,  10)
        self.sub_lap      = self.create_subscription(Int32,       '/lap_count',         self.lap_cb,      10)
        self.sub_overtake = self.create_subscription(Bool,        '/overtaking/active', self.overtake_cb, 10)
        self.sub_enabled  = self.create_subscription(Bool,        '/race/enabled',      self.enabled_cb,  10)

        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info('GapFollowNode ready.')

    def _load_params(self):
        self.car_width        = self.get_parameter('car_width').value
        self.disp_thresh      = self.get_parameter('disparity_threshold').value
        self.bubble_r         = self.get_parameter('bubble_radius').value
        self.spd_obs          = self.get_parameter('speed_with_obstacles').value
        self.spd_no_obs       = self.get_parameter('speed_no_obstacles').value
        self.spd_max          = self.get_parameter('speed_max').value
        self.spd_min          = self.get_parameter('speed_min').value
        self.steer_gain       = self.get_parameter('steer_gain').value
        self.steer_max        = math.radians(self.get_parameter('steer_max_deg').value)
        self.opp_virtual_dist = self.get_parameter('opponent_virtual_dist').value
        self.opp_influence_deg= self.get_parameter('opponent_influence_deg').value
        self.safety_dist      = self.get_parameter('safety_distance').value
        self.obs_removal_lap  = self.get_parameter('obstacle_removal_lap').value

    def scan_cb(self, msg: LaserScan):
        if not self.enabled:
            return
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.clip(ranges, msg.range_min, msg.range_max)
        ranges = np.where(np.isfinite(ranges), ranges, msg.range_max)

        ranges = self._inject_opponent(ranges, msg)
        ranges = self._disparity_extender(ranges, msg)
        best_idx = self._find_best_gap(ranges, msg)
        steer, speed = self._compute_drive(best_idx, ranges, msg)
        self._publish_drive(steer, speed)

    def _inject_opponent(self, ranges, msg):
        if not self.opponent_detected or self.opponent_pose is None:
            return ranges

        ox = self.opponent_pose.pose.position.x
        oy = self.opponent_pose.pose.position.y
        dist = math.hypot(ox, oy)

        if dist < 0.1:
            return ranges
        if dist < self.safety_dist:
            return np.minimum(ranges, np.full_like(ranges, self.safety_dist * 0.5))

        inject_dist  = max(dist - 0.3, self.opp_virtual_dist)
        center_angle = math.atan2(oy, ox)
        half_spread  = math.radians(self.opp_influence_deg / 2.0)
        n = len(ranges)

        for i in range(n):
            beam_angle = msg.angle_min + i * msg.angle_increment
            if abs(beam_angle - center_angle) <= half_spread:
                ranges[i] = min(ranges[i], inject_dist)

        return ranges

    def _disparity_extender(self, ranges, msg):
        n = len(ranges)
        extended = ranges.copy()
        bubble_pts = int(math.ceil(
            math.atan2(self.car_width / 2, 0.01) / max(msg.angle_increment, 1e-6)
        ))
        bubble_pts = min(bubble_pts, n // 4)

        for i in range(1, n):
            diff = abs(ranges[i] - ranges[i - 1])
            if diff > self.disp_thresh:
                closer = i if ranges[i] < ranges[i - 1] else i - 1
                lo = max(0,     closer - bubble_pts)
                hi = min(n - 1, closer + bubble_pts)
                extended[lo:hi + 1] = np.minimum(
                    extended[lo:hi + 1],
                    np.full(hi - lo + 1, ranges[closer])
                )
        return extended

    def _find_best_gap(self, ranges, msg):
        mask = ranges > (self.bubble_r + 0.1)
        best_start = best_end = best_len = 0
        cur_start = cur_len = 0

        for i, free in enumerate(mask):
            if free:
                if cur_len == 0:
                    cur_start = i
                cur_len += 1
                if cur_len > best_len:
                    best_len  = cur_len
                    best_start = cur_start
                    best_end  = i
            else:
                cur_len = 0

        if best_len == 0:
            return len(ranges) // 2

        sub = ranges[best_start:best_end + 1]
        return best_start + int(np.argmax(sub))

    def _compute_drive(self, best_idx, ranges, msg):
        n = len(ranges)
        delta_idx = best_idx - n // 2
        steer = float(np.clip(self.steer_gain * delta_idx * msg.angle_increment,
                               -self.steer_max, self.steer_max))

        base_speed = self.spd_obs if self.lap_count < self.obs_removal_lap else self.spd_no_obs
        if self.overtaking_active:
            base_speed = self.spd_max

        steer_ratio = abs(steer) / self.steer_max
        speed = float(np.clip(base_speed * (1.0 - 0.5 * steer_ratio),
                               self.spd_min, self.spd_max))
        return steer, speed

    def _publish_drive(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.pub_drive.publish(msg)

    def publish_stop(self):
        self._publish_drive(0.0, 0.0)

    def opp_pose_cb(self, msg): self.opponent_pose = msg
    def opp_det_cb(self, msg):
        self.opponent_detected = msg.data
        if not msg.data: self.opponent_pose = None
    def lap_cb(self, msg): self.lap_count = msg.data
    def overtake_cb(self, msg): self.overtaking_active = msg.data
    def enabled_cb(self, msg):
        if self.enabled and not msg.data:
            self.publish_stop()
        self.enabled = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
