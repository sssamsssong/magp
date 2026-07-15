#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('waypoint_file',         '')
        self.declare_parameter('lookahead_distance',    0.8)
        self.declare_parameter('wheelbase',             0.33)
        self.declare_parameter('vgain',                 1.0)
        self.declare_parameter('speed_max',             4.0)
        self.declare_parameter('speed_min',             1.0)
        self.declare_parameter('steer_max_deg',         25.0)
        self.declare_parameter('obstacle_threshold',    1.5)
        self.declare_parameter('front_angle_deg',       30.0)
        self.declare_parameter('use_ftg_with_opponent', True)

        self.waypoint_file   = self.get_parameter('waypoint_file').value
        self.lookahead       = self.get_parameter('lookahead_distance').value
        self.wheelbase       = self.get_parameter('wheelbase').value
        self.vgain           = self.get_parameter('vgain').value
        self.speed_max       = self.get_parameter('speed_max').value
        self.speed_min       = self.get_parameter('speed_min').value
        self.steer_max       = math.radians(self.get_parameter('steer_max_deg').value)
        self.obs_threshold   = self.get_parameter('obstacle_threshold').value
        self.front_angle_deg = self.get_parameter('front_angle_deg').value
        self.use_ftg_opponent= self.get_parameter('use_ftg_with_opponent').value

        self.waypoints        = None
        self.current_x        = 0.0
        self.current_y        = 0.0
        self.current_theta    = 0.0
        self.obstacle_detected = False
        self.opponent_detected = False
        self.use_ftg          = False

        self._load_waypoints()

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_odom    = self.create_subscription(Odometry,  '/ego_racecar/odom',  self.odom_cb,    10)
        self.sub_scan    = self.create_subscription(LaserScan, '/scan',              self.scan_cb,    scan_qos)
        self.sub_opp_det = self.create_subscription(Bool,      '/opponent/detected', self.opp_det_cb, 10)

        self.pub_drive    = self.create_publisher(AckermannDriveStamped, '/drive',               10)
        self.pub_ftg_mode = self.create_publisher(Bool,                  '/pure_pursuit/use_ftg', 10)
        self.pub_markers  = self.create_publisher(MarkerArray,           '/waypoints/markers',   10)

        self.create_timer(0.02, self.control_loop)
        self.create_timer(1.0,  self.publish_markers)

        self.get_logger().info('PurePursuitNode ready.')

    def _load_waypoints(self):
        if not self.waypoint_file:
            self.get_logger().warn('No waypoint file specified!')
            return
        try:
            data = np.loadtxt(self.waypoint_file, delimiter=';', skiprows=3)
            self.waypoints = data[:, [1, 2, 5]]
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            self.waypoints = None

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny, cosy)

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        front  = math.radians(self.front_angle_deg)
        mask   = (np.abs(angles) <= front) & np.isfinite(ranges)
        self.obstacle_detected = float(np.min(ranges[mask])) < self.obs_threshold if mask.any() else False

    def opp_det_cb(self, msg):
        self.opponent_detected = msg.data

    def control_loop(self):
        self.use_ftg = self.obstacle_detected or (self.use_ftg_opponent and self.opponent_detected)
        m = Bool(); m.data = self.use_ftg
        self.pub_ftg_mode.publish(m)

        if self.use_ftg or self.waypoints is None:
            return

        steer, speed = self._pure_pursuit()
        self._publish_drive(steer, speed)

    def _pure_pursuit(self):
        position = np.array([self.current_x, self.current_y])
        wpts     = self.waypoints[:, :2]
        dists    = np.linalg.norm(wpts - position, axis=1)
        nearest  = int(np.argmin(dists))

        target_wp = None
        n = len(wpts)
        for i in range(nearest, nearest + n):
            idx  = i % n
            dist = float(np.linalg.norm(wpts[idx] - position))
            if dist >= self.lookahead:
                target_wp = self.waypoints[idx]
                break

        if target_wp is None:
            return 0.0, self.speed_min

        tx, ty  = target_wp[0], target_wp[1]
        dx      = tx - self.current_x
        dy      = ty - self.current_y
        local_y = -math.sin(self.current_theta) * dx + math.cos(self.current_theta) * dy

        if abs(local_y) < 1e-6:
            steer = 0.0
        else:
            radius = self.lookahead ** 2 / (2.0 * abs(local_y))
            steer  = math.atan(self.wheelbase / radius)
            if local_y < 0:
                steer = -steer

        steer = float(np.clip(steer, -self.steer_max, self.steer_max))
        speed = float(np.clip(self.vgain * target_wp[2], self.speed_min, self.speed_max))
        return steer, speed

    def _publish_drive(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
        msg.drive.speed          = speed
        self.pub_drive.publish(msg)

    def publish_markers(self):
        if self.waypoints is None:
            return
        ma = MarkerArray()
        for i, wp in enumerate(self.waypoints[::10]):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.id              = i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.pose.position.x = float(wp[0])
            m.pose.position.y = float(wp[1])
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8
            ma.markers.append(m)
        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
