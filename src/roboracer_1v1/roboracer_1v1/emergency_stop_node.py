#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')

        self.declare_parameter('min_front_distance', 0.20)
        self.declare_parameter('front_angle_deg',    30.0)
        self.declare_parameter('stop_duration_sec',   2.0)

        self.min_dist  = self.get_parameter('min_front_distance').value
        self.front_deg = self.get_parameter('front_angle_deg').value
        self.stop_dur  = self.get_parameter('stop_duration_sec').value

        self.stopped    = False
        self.stop_until = None

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(LaserScan, '/scan',           self.scan_cb, scan_qos)
        self.sub_ext  = self.create_subscription(Bool,      '/emergency_stop', self.ext_cb,  10)

        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive',        10)
        self.pub_state = self.create_publisher(Bool,                  '/estop/active', 10)

        self.create_timer(0.05, self.update)
        self.get_logger().info('EmergencyStopNode ready.')

    def scan_cb(self, msg):
        front_rad = math.radians(self.front_deg)
        ranges    = np.array(msg.ranges, dtype=np.float32)
        angles    = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        mask      = (np.abs(angles) <= front_rad) & np.isfinite(ranges)
        if mask.any() and np.min(ranges[mask]) < self.min_dist:
            self._trigger('LiDAR front obstacle')

    def ext_cb(self, msg):
        if msg.data: self._trigger('External signal')

    def _trigger(self, reason):
        if not self.stopped:
            self.get_logger().warn(f'E-STOP: {reason}')
        self.stopped    = True
        self.stop_until = self.get_clock().now().nanoseconds + int(self.stop_dur * 1e9)

    def update(self):
        now = self.get_clock().now()
        if self.stopped:
            if now.nanoseconds >= self.stop_until:
                self.stopped = False
                self.get_logger().info('E-STOP released.')
            else:
                cmd = AckermannDriveStamped()
                cmd.header.stamp = now.to_msg()
                cmd.drive.speed  = 0.0
                self.pub_drive.publish(cmd)

        s = Bool(); s.data = self.stopped
        self.pub_state.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()