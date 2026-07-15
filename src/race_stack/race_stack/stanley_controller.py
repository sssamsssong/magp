#!/usr/bin/env python3
"""
StanleyController — Q1/Q2/Q3 unified controller for race_stack.

  q1    : racing-line following, CSV speed, moderate
  q2    : racing-line + LiDAR obstacle avoidance (lateral target shift)
  q3    : racing-line following, aggressive speed (set q3_max higher)

Subscriptions:
  /amcl_pose    PoseWithCovarianceStamped  — AMCL localization
  /odom         Odometry                   — wheel velocity
  /scan         LaserScan                  — Q2 obstacle detection only
  /race/enabled Bool                       — gate from mission_manager

Publications:
  /drive        AckermannDriveStamped
  /waypoint     Marker       — RViz target point
  /obstacle     MarkerArray  — RViz obstacle points (Q2 only)

CSV waypoint format: x, y, speed  (metre, m/s, one row per point)
Boundary files (Q2):  inner_bound.npy, outer_bound.npy  — pixel coords (int32)
                      placed in  maps/<map_name>/  or set bound_dir explicitly.
"""

import csv
import math
import os

import cv2
import numpy as np
import scipy.spatial.distance
import yaml
from PIL import Image

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


# ── helpers ────────────────────────────────────────────────────────────────

def _normalize(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def _yaw(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))


def _load_map(map_yaml_path: str):
    """Load map image + YAML → (map_array [H,W,3], (h,w,res,ox,oy))."""
    cfg  = yaml.safe_load(open(map_yaml_path))
    stem = os.path.splitext(map_yaml_path)[0]
    img_p = None
    for ext in ('.png', '.pgm'):
        if os.path.exists(stem + ext):
            img_p = stem + ext
            break
    if img_p is None:
        raise FileNotFoundError(f'no image found for {stem}')

    img = Image.open(img_p).convert('L').transpose(Image.FLIP_TOP_BOTTOM)
    arr = np.asarray(img, dtype=np.float64)
    arr = np.where(arr > 128, 255.0, 0.0)
    h, w = arr.shape
    res  = float(cfg['resolution'])
    ox   = float(cfg['origin'][0])
    oy   = float(cfg['origin'][1])

    xs, ys = np.meshgrid(np.arange(w) * res + ox,
                         np.arange(h) * res + oy)
    vs = np.where(arr > 0, 0.0, 1.0)   # 1 = occupied wall, 0 = free
    return np.dstack((vs, xs, ys)), (h, w, res, ox, oy)


def _world_to_img(pts: np.ndarray, h, res, ox, oy) -> np.ndarray:
    px = (pts[:, 0] - ox) / res
    py = h - (pts[:, 1] - oy) / res
    return np.vstack((px, py)).T.astype(np.int32)


# ── node ───────────────────────────────────────────────────────────────────

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # ── parameter declarations ─────────────────────────────────────
        self.declare_parameter('mode',            'q1')
        self.declare_parameter('waypoint_file',   '')
        self.declare_parameter('max_speed',        5.0)
        self.declare_parameter('min_speed',        1.0)
        self.declare_parameter('max_steer',        0.36)
        self.declare_parameter('wheelbase',        0.33)
        self.declare_parameter('stanley_gain',     0.8)
        self.declare_parameter('stanley_k',        1.8)
        self.declare_parameter('stanley_ks',       1.3)
        self.declare_parameter('vel_scale',        1.0)
        self.declare_parameter('obstacle_decel',   0.85)
        self.declare_parameter('obs2lane_dist',    0.25)
        self.declare_parameter('interp_scale',     20)
        self.declare_parameter('lookahead_min',    0.5)
        self.declare_parameter('lookahead_max',    2.0)
        self.declare_parameter('lookahead_gain',   7.0)
        # Q2 obstacle detection
        self.declare_parameter('map_yaml',         '')
        self.declare_parameter('bound_dir',        '')
        self.declare_parameter('grid_xmin',        0.0)
        self.declare_parameter('grid_xmax',        5.0)
        self.declare_parameter('grid_ymin',       -2.5)
        self.declare_parameter('grid_ymax',        2.5)
        self.declare_parameter('grid_res',         0.04)
        self.declare_parameter('grid_safe_dist',   0.3)
        self.declare_parameter('bound_ignore_px',  7.5)

        def p(n): return self.get_parameter(n).value

        self.mode       = p('mode')
        self.max_speed  = p('max_speed')
        self.min_speed  = p('min_speed')
        self.max_steer  = p('max_steer')
        self.gain       = p('stanley_gain')
        self.k_cte      = p('stanley_k')
        self.ks         = p('stanley_ks')
        self.vel_scale  = p('vel_scale')
        self.obs_decel  = p('obstacle_decel')
        self.obs2lane   = p('obs2lane_dist')
        self.interp     = int(p('interp_scale'))
        self.L_min      = p('lookahead_min')
        self.L_max      = p('lookahead_max')
        self.L_gain     = p('lookahead_gain')

        # ── waypoints ─────────────────────────────────────────────────
        self.wps = self._load_waypoints(p('waypoint_file'))
        if self.wps is None:
            self.get_logger().error(
                f"waypoints not found: '{p('waypoint_file')}'\n"
                "Place your CSV (x,y,speed) in race_stack/waypoints/ and set "
                "the waypoints key in race_script.yaml.")
        else:
            self.get_logger().info(
                f'[Stanley/{self.mode}] {len(self.wps)} waypoints loaded')

        # ── state ─────────────────────────────────────────────────────
        self.enabled        = True
        self.pos            = None
        self.yaw            = 0.0
        self.speed          = 0.0
        self.lane_free      = True
        self.free_votes: list[bool] = []
        self.all_on_track: list[np.ndarray] = []

        # ── Q2: map image + polygon bounds ────────────────────────────
        self.map_arr        = None
        self.map_meta       = None
        self.inner_bound    = None
        self.outer_bound    = None

        if self.mode == 'q2':
            map_yaml  = p('map_yaml')
            bound_dir = p('bound_dir') or (
                os.path.join(os.path.dirname(map_yaml),
                             os.path.splitext(os.path.basename(map_yaml))[0])
                if map_yaml else '')

            if map_yaml:
                try:
                    self.map_arr, self.map_meta = _load_map(map_yaml)
                    self.get_logger().info(f'map loaded for obstacle detection')
                except Exception as e:
                    self.get_logger().error(f'map load failed: {e}')

            for fname, attr in [('inner_bound.npy', 'inner_bound'),
                                 ('outer_bound.npy', 'outer_bound')]:
                fpath = os.path.join(bound_dir, fname)
                if os.path.exists(fpath):
                    setattr(self, attr, np.load(fpath))
                else:
                    self.get_logger().warn(
                        f'{fname} not found in {bound_dir} — '
                        'obstacle avoidance will run without polygon test')

        # ── subscriptions / publishers ─────────────────────────────────
        self.create_subscription(Bool,
            '/race/enabled', self._enabled_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped,
            '/amcl_pose', self._pose_cb, 10)
        self.create_subscription(Odometry,
            '/odom', self._odom_cb, 10)
        if self.mode == 'q2':
            self.create_subscription(LaserScan,
                '/scan', self._scan_cb, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive',    10)
        self.wp_pub    = self.create_publisher(Marker,                 '/waypoint', 10)
        self.obs_pub   = self.create_publisher(MarkerArray,            '/obstacle', 10)

        self.get_logger().info(f'StanleyController ready [mode={self.mode}]')

    # ── event callbacks ────────────────────────────────────────────────────

    def _enabled_cb(self, msg: Bool):
        if self.enabled and not msg.data:
            self._publish_cmd(0.0, 0.0)
        self.enabled = msg.data

    def _odom_cb(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x

    # ── main control loop (triggered by AMCL at ~10 Hz) ───────────────────

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        if not self.enabled or self.wps is None:
            return

        px  = msg.pose.pose.position.x
        py  = msg.pose.pose.position.y
        yaw = _yaw(msg.pose.pose.orientation)
        self.pos = np.array([px, py])
        self.yaw = yaw
        speed    = abs(self.speed)

        # ── adaptive lookahead: L = (Lmax-Lmin)/Lgain * v + Lmin ──────
        L = float(np.clip(
            (self.L_max - self.L_min) / self.L_gain * speed + self.L_min,
            self.L_min, self.L_max))

        # ── nearest waypoint ───────────────────────────────────────────
        wps = self.wps
        n   = len(wps)
        d   = np.hypot(wps[:, 0] - px, wps[:, 1] - py)
        nearest = int(np.argmin(d))

        # ── walk forward until Euclidean dist from nearest > L ────────
        traj_d  = np.hypot(wps[:, 0] - wps[nearest, 0],
                            wps[:, 1] - wps[nearest, 1])
        seg_end = nearest
        for _ in range(n):
            if traj_d[seg_end] > L:
                break
            seg_end = (seg_end + 1) % n
        seg_beg = (seg_end - 1 + n) % n

        # ── interpolate exact target at distance L ─────────────────────
        xs = np.linspace(wps[seg_beg, 0], wps[seg_end, 0], self.interp)
        ys = np.linspace(wps[seg_beg, 1], wps[seg_end, 1], self.interp)
        i_best = int(np.argmin(np.abs(np.hypot(xs - px, ys - py) - L)))
        target = np.array([xs[i_best], ys[i_best]])

        # ── Q2: shift target laterally away from obstacle ──────────────
        if self.mode == 'q2':
            target = self._obstacle_shift(target, px, py, yaw)

        # ── Stanley steering ───────────────────────────────────────────
        closest  = wps[nearest, :2]
        path_yaw = math.atan2(target[1] - closest[1], target[0] - closest[0])
        cte_size = float(np.hypot(px - closest[0], py - closest[1]))
        cte_sign = float(np.sign(np.cross(
            [math.cos(path_yaw), math.sin(path_yaw)],
            [px - closest[0],    py - closest[1]])))
        cte     = cte_size * cte_sign
        yaw_err = _normalize(path_yaw - yaw)

        v_safe = max(speed, 0.5)   # avoid division by near-zero
        steer  = self.gain * (yaw_err
                              - math.atan2(self.k_cte * cte, v_safe + self.ks))
        steer  = float(np.clip(steer, -self.max_steer, self.max_steer))

        # ── target speed from CSV ──────────────────────────────────────
        cmd_v = float(wps[nearest, 2]) * self.vel_scale
        if self.mode == 'q2' and not self.lane_free:
            cmd_v *= self.obs_decel
        cmd_v = float(np.clip(cmd_v, self.min_speed, self.max_speed))

        self._publish_cmd(steer, cmd_v)
        self._pub_wp_marker(target)

    # ── Q2 obstacle avoidance ──────────────────────────────────────────────

    def _obstacle_shift(self, target: np.ndarray,
                        px: float, py: float, yaw: float) -> np.ndarray:
        # update lane_free: need 5+ free votes in last 10
        if len(self.free_votes) > 10:
            self.lane_free = sum(self.free_votes[-10:]) >= 5

        if not self.lane_free and self.all_on_track:
            fwd  = np.array([math.cos(yaw), math.sin(yaw)])
            left = np.array([-fwd[1], fwd[0]])
            obs_c   = np.mean(self.all_on_track, axis=0)
            lateral = float((obs_c - np.array([px, py])) @ left)
            sign    = -np.sign(lateral) if abs(lateral) > 0.05 else -1.0
            target  = target + sign * 0.2 * left
        return target

    def _scan_cb(self, msg: LaserScan):
        if self.pos is None:
            return

        p = self.get_parameter
        xmin = p('grid_xmin').value;   xmax = p('grid_xmax').value
        ymin = p('grid_ymin').value;   ymax = p('grid_ymax').value
        res  = p('grid_res').value;    safe = p('grid_safe_dist').value
        ign  = p('bound_ignore_px').value

        ranges = np.clip(msg.ranges, msg.range_min, msg.range_max)

        # build occupied grid in vehicle frame
        nx = int((xmax - xmin) / res) + 1
        ny = int((ymax - ymin) / res) + 1
        gx = np.linspace(xmin, xmax, nx)
        gy = np.linspace(ymin, ymax, ny)
        gy, gx = np.meshgrid(gy, gx)

        rho = np.sqrt(gx**2 + gy**2)
        phi = np.arctan2(gy, gx)
        idx = np.clip(
            ((phi - msg.angle_min) / msg.angle_increment).astype(int),
            0, len(ranges) - 1)
        occ = (np.abs(rho - ranges[idx]) < safe).ravel()

        gxf = gx.ravel()[occ]
        gyf = gy.ravel()[occ]

        if len(gxf) == 0:
            self.all_on_track = []
            self.free_votes.append(True)
            return

        # rotate + translate to world frame
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        R  = np.array([[cy, -sy], [sy, cy]])
        pts_world = (R @ np.vstack((gxf, gyf))).T + self.pos

        self._classify_obstacles(pts_world, ign)

    def _classify_obstacles(self, pts_world: np.ndarray, ign: float):
        # without bound files → treat all grid hits as lane-blocking
        if self.inner_bound is None or self.map_arr is None:
            if len(pts_world) > 0:
                d_min = float(scipy.spatial.distance.cdist(
                    self.wps[:, :2], pts_world).min())
                self.all_on_track = list(pts_world)
                self.free_votes.append(d_min > self.obs2lane)
            else:
                self.all_on_track = []
                self.free_votes.append(True)
            return

        h, w, res, ox, oy = self.map_meta
        pts_img = _world_to_img(pts_world, h, res, ox, oy)

        on_track = []
        for i, (ix, iy) in enumerate(pts_img):
            if not (0 <= ix < w and 0 <= iy < h):
                continue
            io = cv2.pointPolygonTest(self.outer_bound, (float(ix), float(iy)), True)
            ii = cv2.pointPolygonTest(self.inner_bound, (float(ix), float(iy)), True)
            if io > ign and ii < -ign:
                on_track.append(pts_world[i])

        self.all_on_track = on_track

        if not on_track:
            self.free_votes.append(True)
            return

        d_min = float(scipy.spatial.distance.cdist(
            self.wps[:, :2], np.array(on_track)).min())
        self.free_votes.append(d_min > self.obs2lane)
        self._pub_obstacle_markers(on_track)

    # ── publishers / helpers ───────────────────────────────────────────────

    def _publish_cmd(self, steer: float, speed: float):
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
        msg.drive.speed          = speed
        self.drive_pub.publish(msg)

    def _pub_wp_marker(self, target: np.ndarray):
        m = Marker()
        m.header.frame_id    = 'map'
        m.ns = 'target_wp';  m.id = 0
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position.x    = float(target[0])
        m.pose.position.y    = float(target[1])
        m.pose.orientation.w = 1.0
        m.color.r = m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.lifetime.nanosec   = int(1e8)
        self.wp_pub.publish(m)

    def _pub_obstacle_markers(self, pts: list):
        arr = MarkerArray()
        for i, pt in enumerate(pts[:30]):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'obstacle'; m.id = i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.color.r = 1.0; m.color.a = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.lifetime.nanosec = int(1e8)
            arr.markers.append(m)
        self.obs_pub.publish(arr)

    def _load_waypoints(self, path: str):
        if not path or not os.path.exists(path):
            return None
        pts = []
        try:
            with open(path) as f:
                for row in csv.reader(f):
                    if not row or row[0].strip().startswith('#'):
                        continue
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        v = float(row[2]) if len(row) > 2 else 3.0
                        pts.append((x, y, v))
                    except (ValueError, IndexError):
                        continue
        except OSError:
            return None
        return np.array(pts) if pts else None


# ── entry point ────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = StanleyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
