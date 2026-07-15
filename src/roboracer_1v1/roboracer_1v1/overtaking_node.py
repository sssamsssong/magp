#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from ackermann_msgs.msg import AckermannDriveStamped


class OvertakingNode(Node):
    def __init__(self):
        super().__init__('overtaking_node')

        self.declare_parameter('engage_distance',        4.0)
        self.declare_parameter('side_gap_threshold',     1.2)
        self.declare_parameter('overtake_steer_offset',  0.18)
        self.declare_parameter('overtake_duration_sec',  3.0)
        self.declare_parameter('cooldown_sec',           5.0)
        self.declare_parameter('side_scan_angle_deg',    60.0)
        self.declare_parameter('side_scan_start_deg',    45.0)

        self.engage_dist    = self.get_parameter('engage_distance').value
        self.gap_thresh     = self.get_parameter('side_gap_threshold').value
        self.steer_offset   = self.get_parameter('overtake_steer_offset').value
        self.overtake_dur   = self.get_parameter('overtake_duration_sec').value
        self.cooldown       = self.get_parameter('cooldown_sec').value
        self.side_angle_deg = self.get_parameter('side_scan_angle_deg').value
        self.side_start_deg = self.get_parameter('side_scan_start_deg').value

        self.opponent_dist         = np.inf
        self.opponent_detected     = False
        self.overtaking            = False
        self.overtake_direction    = 0
        self.overtake_start_time   = None
        self.last_overtake_end_time= None
        self.latest_scan           = None

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan     = self.create_subscription(LaserScan, '/scan',              self.scan_cb, scan_qos)
        self.sub_opp_dist = self.create_subscription(Float32,   '/opponent/distance', self.dist_cb, 10)
        self.sub_opp_det  = self.create_subscription(Bool,      '/opponent/detected', self.det_cb,  10)

        self.pub_active = self.create_publisher(Bool,    '/overtaking/active',       10)
        self.pub_offset = self.create_publisher(Float32, '/overtaking/steer_offset', 10)
        self.pub_dir    = self.create_publisher(Float32, '/overtaking/direction',    10)

        self.create_timer(0.05, self.update)
        self.get_logger().info('OvertakingNode ready.')

    def scan_cb(self, msg): self.latest_scan = msg
    def dist_cb(self, msg): self.opponent_dist = msg.data
    def det_cb(self, msg):
        self.opponent_detected = msg.data
        if not msg.data: self.opponent_dist = np.inf

    def update(self):
        now = self.get_clock().now()

        if self.overtaking and self.overtake_start_time is not None:
            elapsed = (now - self.overtake_start_time).nanoseconds * 1e-9
            if elapsed >= self.overtake_dur:
                self._end_overtake(now)
                return
            self._publish_active(True)
            return

        if self.last_overtake_end_time is not None:
            if (now - self.last_overtake_end_time).nanoseconds * 1e-9 < self.cooldown:
                self._publish_active(False)
                return

        if not self.opponent_detected or self.opponent_dist > self.engage_dist:
            self._publish_active(False)
            return

        if self.latest_scan is None:
            return

        left_gap, right_gap = self._measure_side_gaps(self.latest_scan)
        direction = self._decide_direction(left_gap, right_gap)

        if direction == 0:
            self._publish_active(False)
            return

        self.overtaking          = True
        self.overtake_direction  = direction
        self.overtake_start_time = now
        self.get_logger().info(
            f'OVERTAKE → {"LEFT" if direction > 0 else "RIGHT"} '
            f'(dist={self.opponent_dist:.2f} L={left_gap:.2f} R={right_gap:.2f})'
        )
        self._publish_active(True)

    def _measure_side_gaps(self, msg):
        start = math.radians(self.side_start_deg)
        end   = math.radians(self.side_start_deg + self.side_angle_deg)
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        l_mask = (angles >= start)  & (angles <= end)  & np.isfinite(ranges)
        r_mask = (angles <= -start) & (angles >= -end) & np.isfinite(ranges)

        left  = float(np.mean(ranges[l_mask])) if l_mask.any() else 0.0
        right = float(np.mean(ranges[r_mask])) if r_mask.any() else 0.0
        return left, right

    def _decide_direction(self, left, right):
        l_ok = left  >= self.gap_thresh
        r_ok = right >= self.gap_thresh
        if l_ok and r_ok:  return 1 if left >= right else -1
        elif l_ok:         return 1
        elif r_ok:         return -1
        else:              return 0

    def _end_overtake(self, now):
        self.overtaking              = False
        self.overtake_direction      = 0
        self.last_overtake_end_time  = now
        self.get_logger().info('OVERTAKE END')
        self._publish_active(False)

    def _publish_active(self, active):
        b = Bool(); b.data = active
        self.pub_active.publish(b)
        o = Float32(); o.data = float(self.steer_offset * self.overtake_direction) if active else 0.0
        self.pub_offset.publish(o)
        d = Float32(); d.data = float(self.overtake_direction)
        self.pub_dir.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = OvertakingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()