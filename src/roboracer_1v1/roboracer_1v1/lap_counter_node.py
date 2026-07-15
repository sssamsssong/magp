#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32


class LapCounterNode(Node):
    def __init__(self):
        super().__init__('lap_counter_node')

        self.declare_parameter('finish_line_x',       0.0)
        self.declare_parameter('finish_line_y',       0.0)
        self.declare_parameter('min_lap_time_sec',    5.0)
        self.declare_parameter('trigger_radius',      0.6)

        self.fx      = self.get_parameter('finish_line_x').value
        self.fy      = self.get_parameter('finish_line_y').value
        self.min_lap = self.get_parameter('min_lap_time_sec').value
        self.radius  = self.get_parameter('trigger_radius').value

        self.lap_count     = 0
        self.in_zone       = False
        self.last_lap_time = self.get_clock().now()

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub_lap  = self.create_publisher(Int32, '/lap_count', 10)
        self.get_logger().info('LapCounterNode ready.')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        in_zone_now = math.hypot(x - self.fx, y - self.fy) < self.radius

        if in_zone_now and not self.in_zone:
            elapsed = (self.get_clock().now() - self.last_lap_time).nanoseconds * 1e-9
            if elapsed >= self.min_lap:
                self.lap_count += 1
                self.last_lap_time = self.get_clock().now()
                m = Int32(); m.data = self.lap_count
                self.pub_lap.publish(m)
                self.get_logger().info(f'LAP {self.lap_count} ({elapsed:.1f}s)')

        self.in_zone = in_zone_now


def main(args=None):
    rclpy.init(args=args)
    node = LapCounterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()