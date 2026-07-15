#!/usr/bin/env python3
"""
하이브리드 경로 추종 컨트롤러
- APP  (Adaptive Pure Pursuit) : 타이트 코너 + 경로 이탈 복귀
- Stanley Controller            : 완만한 커브 구간
- LQR  (Linear Quadratic Regulator): 직선 구간
- FTG  (Follow the Gap)         : 장애물 비상 탈출
- 쿠빅 스플라인 기반 곡률·헤딩 사전 계산
- 역방향 스윕 속도 프로파일 사전 계산
"""

import rclpy
from rclpy.node import Node
import numpy as np
import csv
import os
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# WAYPOINTS_MAP 과 _MAPS_DIR 는 pure_pursuit_node 에서 공유
# setup_sim.py 는 pure_pursuit_node.py 만 수정하면 두 컨트롤러 모두 동일 맵 사용
from f1tenth_racing.pure_pursuit_node import WAYPOINTS_MAP, _MAPS_DIR

WAYPOINTS_PATH = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}_waypoints.csv')


class HybridController(Node):
    def __init__(self):
        super().__init__('hybrid_controller')
        self._near_idx   = 0
        self._initialized = False

        # ── 차량 파라미터 ───────────────────────────────────────────────────
        self.WHEELBASE   = 0.33
        self.MAX_STEER   = 0.4
        self.SPEED_SCALE = 1.1057

        # ── APP (Adaptive Pure Pursuit) 파라미터 ────────────────────────────
        # lookahead = LA_BASE + LA_K*v - LA_K_CURV*κ
        self.LA_BASE     = 0.5931
        self.LA_K        = 0.1234   # 속도 비례 증가
        self.LA_K_CURV   = 0.8     # 곡률 비례 감소 (코너일수록 짧아짐)

        # ── Stanley Controller 파라미터 ─────────────────────────────────────
        self.STANLEY_K   = 0.5     # 횡오차 → 조향 게인

        # ── LQR 파라미터 (상태: [cte, heading_err]) ─────────────────────────
        self.LQR_Q1      = 1.0     # CTE 페널티
        self.LQR_Q2      = 0.5     # 헤딩 오차 페널티
        self.LQR_R       = 1.0     # 조향 입력 페널티

        # ── 제어기 전환 임계값 ──────────────────────────────────────────────
        self.HIGH_CURV          = 0.3    # 1/m 이상 → APP  (반경 ~3.3m)
        self.LOW_CURV           = 0.05   # 1/m 이하 → LQR  (반경 ~20m)
        self.CTE_RECOVER_THRESH = 0.3    # m: 횡오차 이상이면 APP 강제
        self.BLEND_MARGIN       = 0.03   # 전환 경계 혼합 범위

        # ── 제동 파라미터 ───────────────────────────────────────────────────
        self.A_BRAKE     = 9.4675

        # ── 조향 저역통과 필터 ──────────────────────────────────────────────
        self.STEER_ALPHA = 0.40
        self._steer_prev = 0.0

        # ── 비상 회피 파라미터 ──────────────────────────────────────────────
        self.WALL_SIDE_THRESH      = 0.5
        self.WALL_BIAS_GAIN        = 0.4
        self.COLLISION_THRESH      = 0.25
        self.BRAKE_START_DIST      = 2.0
        self.OBSTACLE_TRIGGER_DIST = 1.2   # 전방 장애물 즉시 FTG 전환 거리
        self.REVERSE_SPEED         = 0.8
        self.REVERSE_DURATION      = 3.5

        # ── Stuck → FTG 파라미터 ───────────────────────────────────────────
        self.STUCK_TIME        = 3.0
        self.STUCK_DIST_THRESH = 0.15

        # ── FTG 파라미터 ────────────────────────────────────────────────────
        self.FTG_GAP_THRESH  = 0.5
        self.FTG_BUBBLE_R    = 0.3
        self.FTG_SPEED_MIN   = 0.5
        self.FTG_SPEED_MAX   = 3.0
        self.FTG_SPEED_GAIN  = 1.0
        self.FTG_STEER_GAIN  = 1.0

        # ── 상태 머신 ───────────────────────────────────────────────────────
        self._drive_state          = 'NORMAL'
        self._reverse_start        = None
        self._reverse_return_state = 'NORMAL'
        self._scan_data            = None
        self._pos_history          = []
        self._cur_speed            = 0.0

        # ── 웨이포인트 로드 ─────────────────────────────────────────────────
        self.wp = self._load_waypoints(WAYPOINTS_PATH)
        self.get_logger().info(f'웨이포인트 {len(self.wp)}개 로드 완료')

        # ── 사전 계산 ───────────────────────────────────────────────────────
        self._precompute_spline_data()
        self._precompute_speed_profile()
        self._precompute_lqr_gains()
        self._save_controller_map()

        # ── ROS 인터페이스 ──────────────────────────────────────────────────
        self.drive_pub  = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.path_pub   = self.create_publisher(Path, '/waypoints_path', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 1)
        self.create_subscription(Odometry, '/ego_racecar/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_timer(1.0, self._publish_path)
        self._path_published = False

        input('Enter 키를 누르면 주행을 시작합니다...')
        self._pos_history.clear()

    # ── 웨이포인트 로드 ────────────────────────────────────────────────────
    def _load_waypoints(self, path):
        pts = []
        with open(path) as f:
            for row in csv.DictReader(f):
                pts.append([float(row['x']), float(row['y']), float(row['speed'])])
        return np.array(pts)

    # ── 사전 계산: 쿠빅 스플라인 → 곡률·헤딩 배열 ────────────────────────
    def _precompute_spline_data(self):
        from scipy.interpolate import CubicSpline

        pts = self.wp[:, :2]
        n   = len(pts)

        diffs = np.diff(pts, axis=0)
        ds    = np.linalg.norm(diffs, axis=1)
        s     = np.concatenate([[0.0], np.cumsum(ds)])

        s_ext = np.append(s, s[-1] + np.linalg.norm(pts[0] - pts[-1]))
        x_ext = np.append(pts[:, 0], pts[0, 0])
        y_ext = np.append(pts[:, 1], pts[0, 1])

        cs_x = CubicSpline(s_ext, x_ext, bc_type='periodic')
        cs_y = CubicSpline(s_ext, y_ext, bc_type='periodic')

        dx  = cs_x(s, 1);  dy  = cs_y(s, 1)
        ddx = cs_x(s, 2);  ddy = cs_y(s, 2)

        denom = np.maximum((dx**2 + dy**2) ** 1.5, 1e-6)
        self._curvature = np.abs(dx * ddy - dy * ddx) / denom
        self._heading   = np.arctan2(dy, dx)

        self.get_logger().info(
            f'스플라인 사전 계산: 곡률 max={self._curvature.max():.3f} '
            f'mean={self._curvature.mean():.3f} 1/m')

    # ── 사전 계산: 역방향 스윕 속도 프로파일 ─────────────────────────────
    def _precompute_speed_profile(self):
        n  = len(self.wp)
        ds = np.array([
            np.linalg.norm(self.wp[(i + 1) % n, :2] - self.wp[i, :2])
            for i in range(n)
        ])

        self._v_profile = self.wp[:, 2].copy()

        for _ in range(3):
            for i in range(n - 1, -1, -1):
                j = (i + 1) % n
                v_lim = np.sqrt(self._v_profile[j] ** 2 + 2.0 * self.A_BRAKE * ds[i])
                self._v_profile[i] = min(self._v_profile[i], v_lim)

        self.get_logger().info(
            f'속도 프로파일: {self._v_profile.min():.2f}~{self._v_profile.max():.2f} m/s')

    # ── 사전 계산: LQR 게인 테이블 ────────────────────────────────────────
    def _precompute_lqr_gains(self):
        try:
            from scipy.linalg import solve_continuous_are
        except ImportError:
            self.get_logger().warn('scipy 없음 → LQR 비활성화 (Stanley 대체)')
            self._lqr_available = False
            return

        self._lqr_available = True
        speeds = np.linspace(0.3, 9.0, 30)
        Q = np.diag([self.LQR_Q1, self.LQR_Q2])
        R = np.array([[self.LQR_R]])

        v_list, k_list = [], []
        for v in speeds:
            A = np.array([[0.0, v],
                          [0.0, 0.0]])
            B = np.array([[0.0],
                          [v / self.WHEELBASE]])
            try:
                P = solve_continuous_are(A, B, Q, R)
                K = (np.linalg.inv(R) @ B.T @ P)[0]
                v_list.append(v)
                k_list.append(K)
            except Exception:
                pass

        self._lqr_speeds = np.array(v_list)
        self._lqr_K      = np.array(k_list)
        self.get_logger().info(f'LQR 게인 테이블: {len(v_list)}개 속도값 계산 완료')

    # ── 제어기 분포 이미지 저장 ────────────────────────────────────────────
    def _save_controller_map(self):
        try:
            import yaml
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from PIL import Image as PILImage

            yaml_path = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}.yaml')
            with open(yaml_path) as f:
                cfg = yaml.safe_load(f)
            resolution = float(cfg.get('resolution', 0.05))
            origin_raw = cfg.get('origin', [-10.0, -10.0, 0.0])
            ox, oy = float(origin_raw[0]), float(origin_raw[1])

            img_path = None
            for ext in ['.png', '.pgm']:
                p = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}{ext}')
                if os.path.exists(p):
                    img_path = p
                    break
            if img_path is None:
                self.get_logger().warn('제어기 맵: 맵 이미지 없음 → 저장 생략')
                return

            img  = np.array(PILImage.open(img_path).convert('L'))
            h_px = img.shape[0]

            wx = self.wp[:, 0];  wy = self.wp[:, 1]
            px = (wx - ox) / resolution
            py = h_px - (wy - oy) / resolution

            ctrl = np.where(
                self._curvature >= self.HIGH_CURV, 0,
                np.where(self._curvature <= self.LOW_CURV, 2, 1))
            n_app     = int((ctrl == 0).sum())
            n_stanley = int((ctrl == 1).sum())
            n_lqr     = int((ctrl == 2).sum())

            COLORS = {0: ('red',   'APP'),
                      1: ('limegreen', 'Stanley'),
                      2: ('dodgerblue', 'LQR')}

            fig, (ax_map, ax_curv) = plt.subplots(
                2, 1, figsize=(12, 14),
                gridspec_kw={'height_ratios': [4, 1]})

            ax_map.imshow(img, cmap='gray', origin='upper', alpha=0.7)
            for cid, (color, label) in COLORS.items():
                mask = ctrl == cid
                if mask.any():
                    ax_map.scatter(px[mask], py[mask],
                                   c=color, s=8, label=label,
                                   linewidths=0, zorder=3)
            ax_map.legend(loc='lower right', fontsize=12, framealpha=0.9)
            ax_map.set_title(
                f'{WAYPOINTS_MAP}  Controller Distribution\n'
                f'APP {n_app}pts  Stanley {n_stanley}pts  LQR {n_lqr}pts',
                fontsize=13)
            ax_map.axis('off')

            idx = np.arange(len(self.wp))
            ax_curv.plot(idx, self._curvature, color='gray', linewidth=1.0, label='곡률')
            ax_curv.axhline(self.HIGH_CURV, color='red',        linestyle='--',
                            linewidth=1.2, label=f'HIGH_CURV={self.HIGH_CURV}')
            ax_curv.axhline(self.LOW_CURV,  color='dodgerblue', linestyle='--',
                            linewidth=1.2, label=f'LOW_CURV={self.LOW_CURV}')

            ax_curv.fill_between(idx, self.HIGH_CURV, self._curvature,
                                 where=self._curvature >= self.HIGH_CURV,
                                 color='red', alpha=0.2)
            ax_curv.fill_between(idx, 0, self._curvature,
                                 where=self._curvature <= self.LOW_CURV,
                                 color='dodgerblue', alpha=0.2)
            ax_curv.fill_between(idx, self.LOW_CURV, self._curvature,
                                 where=(self._curvature > self.LOW_CURV) &
                                       (self._curvature < self.HIGH_CURV),
                                 color='limegreen', alpha=0.2)

            ax_curv.set_xlim(0, len(self.wp))
            ax_curv.set_ylim(0, self._curvature.max() * 1.15)
            ax_curv.set_xlabel('Waypoint Index', fontsize=10)
            ax_curv.set_ylabel('Curvature (1/m)', fontsize=10)
            ax_curv.legend(loc='upper right', fontsize=9, framealpha=0.9)
            ax_curv.grid(True, alpha=0.3)

            plt.tight_layout()
            out_path = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}_controller.jpg')
            plt.savefig(out_path, dpi=150, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f'제어기 맵 저장: {out_path}')

        except Exception as e:
            self.get_logger().warn(f'제어기 맵 저장 실패: {e}')

    # ── 경로 시각화 (RViz) ─────────────────────────────────────────────────
    def _publish_path(self):
        now = self.get_clock().now().to_msg()

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp    = now
        for x, y, _ in self.wp:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        path_msg.poses.append(path_msg.poses[0])
        self.path_pub.publish(path_msg)

        v_min = self.wp[:, 2].min()
        v_max = self.wp[:, 2].max()
        v_rng = max(v_max - v_min, 0.1)
        markers = MarkerArray()
        for i, (x, y, v) in enumerate(self.wp):
            t = (v - v_min) / v_rng
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = now
            m.ns = 'waypoints'; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = x; m.pose.position.y = y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color = ColorRGBA(r=1.0 - t, g=t, b=0.0, a=0.8)
            markers.markers.append(m)
        self.marker_pub.publish(markers)

        if not self._path_published:
            self.get_logger().info('경로 시각화 완료')
            self._path_published = True

    # ── LiDAR 콜백 + 충돌 비상 제동 ──────────────────────────────────────
    def _scan_cb(self, msg):
        self._scan_data = msg
        if self._drive_state == 'REVERSING':
            return
        front_min = self._front_min_dist(msg)
        if front_min < self.COLLISION_THRESH:
            self.get_logger().warn(
                f'충돌 감지! 전방 최소 거리 {front_min:.2f}m → 후진 시작')
            self._reverse_return_state = self._drive_state
            self._drive_state          = 'REVERSING'
            self._reverse_start        = self.get_clock().now()

    def _front_min_dist(self, msg):
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        ranges = np.array(msg.ranges, dtype=float)
        valid  = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        front  = valid & (np.abs(angles) < np.radians(30))
        return float(ranges[front].min()) if front.any() else float('inf')

    def _side_bias_correction(self):
        """좌/우 벽 거리 불균형 시 반대 방향 조향 보정값(rad) 반환."""
        if self._scan_data is None:
            return 0.0
        msg    = self._scan_data
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        ranges = np.array(msg.ranges, dtype=float)
        valid  = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)

        left_mask  = valid & (angles >  np.radians(60))  & (angles <  np.radians(120))
        right_mask = valid & (angles > -np.radians(120)) & (angles < -np.radians(60))

        left_dist  = float(np.median(ranges[left_mask]))  if left_mask.any()  else self.WALL_SIDE_THRESH
        right_dist = float(np.median(ranges[right_mask])) if right_mask.any() else self.WALL_SIDE_THRESH

        correction = 0.0
        if left_dist < self.WALL_SIDE_THRESH:
            correction += self.WALL_BIAS_GAIN * (self.WALL_SIDE_THRESH - left_dist) / self.WALL_SIDE_THRESH
        if right_dist < self.WALL_SIDE_THRESH:
            correction -= self.WALL_BIAS_GAIN * (self.WALL_SIDE_THRESH - right_dist) / self.WALL_SIDE_THRESH
        return float(np.clip(correction, -0.3, 0.3))

    def _handle_reversing(self):
        elapsed = (self.get_clock().now() - self._reverse_start).nanoseconds * 1e-9
        if elapsed >= self.REVERSE_DURATION:
            self._drive_state = self._reverse_return_state
            self._initialized = False
            self._pos_history.clear()
            self.get_logger().info(f'후진 완료 → {self._drive_state} 복귀')
            return
        cmd = AckermannDriveStamped()
        cmd.drive.speed          = -self.REVERSE_SPEED
        cmd.drive.steering_angle = 0.0
        self.drive_pub.publish(cmd)

    def _check_stuck(self, px, py):
        """(is_stuck, displacement_m) 반환."""
        now_ns = self.get_clock().now().nanoseconds
        self._pos_history.append((now_ns, px, py))
        cutoff_ns = now_ns - int(self.STUCK_TIME * 1e9)
        self._pos_history = [(t, x, y) for t, x, y in self._pos_history if t >= cutoff_ns]
        if len(self._pos_history) < 2:
            return False, 0.0
        oldest_ns, ox, oy = self._pos_history[0]
        if (now_ns - oldest_ns) * 1e-9 < self.STUCK_TIME * 0.9:
            return False, 0.0
        disp = float(np.hypot(px - ox, py - oy))
        return disp < self.STUCK_DIST_THRESH, disp

    def _follow_the_gap(self):
        if self._scan_data is None:
            return
        msg    = self._scan_data
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        ranges = np.array(msg.ranges, dtype=float)

        front = np.abs(angles) < np.radians(90)
        r     = np.where(np.isfinite(ranges), ranges, msg.range_max)
        r     = np.clip(r, 0.0, msg.range_max)
        r[~front] = 0.0

        front_valid = front & np.isfinite(ranges) & (ranges > msg.range_min)
        if front_valid.any():
            min_idx     = int(np.argmin(np.where(front_valid, ranges, np.inf)))
            bubble_half = max(1, int(self.FTG_BUBBLE_R / max(msg.angle_increment, 1e-6)))
            r[max(0, min_idx - bubble_half): min_idx + bubble_half + 1] = 0.0

        best = (0, 0, 0)
        cur_start, cur_len = None, 0
        for i, rv in enumerate(r):
            if rv > self.FTG_GAP_THRESH:
                if cur_start is None:
                    cur_start = i
                cur_len += 1
                if cur_len > best[2]:
                    best = (cur_start, i, cur_len)
            else:
                cur_start, cur_len = None, 0

        gs, ge, gl = best
        if gl == 0:
            cmd = AckermannDriveStamped()
            cmd.drive.speed = 0.0
            self.drive_pub.publish(cmd)
            return

        best_i       = gs + int(np.argmax(r[gs:ge + 1]))
        target_angle = float(angles[best_i])

        front_r   = ranges[front_valid]
        front_min = float(front_r.min()) if len(front_r) > 0 else msg.range_max
        speed     = float(np.clip(front_min * self.FTG_SPEED_GAIN,
                                  self.FTG_SPEED_MIN, self.FTG_SPEED_MAX))

        steer = float(np.clip(target_angle * self.FTG_STEER_GAIN, -self.MAX_STEER, self.MAX_STEER))
        steer = self.STEER_ALPHA * self._steer_prev + (1.0 - self.STEER_ALPHA) * steer
        self._steer_prev = steer

        self.get_logger().info(
            f'[FTG] 갭[{gs}:{ge}] 목표각={np.degrees(target_angle):.1f}°  '
            f'전방최소={front_min:.2f}m  spd={speed:.2f}')

        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = steer
        cmd.drive.speed          = speed
        self.drive_pub.publish(cmd)

    # ── CTE + 헤딩 오차 계산 ──────────────────────────────────────────────
    def _compute_cte_and_heading_err(self, px, py, yaw, near_idx):
        """
        cte        : 양수 = 차량이 경로 왼쪽
        heading_err: 양수 = 차량 헤딩이 경로 방향보다 왼쪽
        """
        n  = len(self.wp)
        p1 = self.wp[near_idx, :2]
        p2 = self.wp[(near_idx + 1) % n, :2]

        tangent = p2 - p1
        t_len   = np.linalg.norm(tangent)
        if t_len < 1e-6:
            return 0.0, 0.0
        tangent /= t_len
        normal  = np.array([-tangent[1], tangent[0]])

        cte         = float(np.dot(np.array([px, py]) - p1, normal))
        path_yaw    = self._heading[near_idx]
        heading_err = float(np.arctan2(np.sin(yaw - path_yaw),
                                       np.cos(yaw - path_yaw)))
        return cte, heading_err

    # ── APP 조향각 계산 ────────────────────────────────────────────────────
    def _steer_app(self, pos, yaw, near_idx):
        """곡률 적응형 lookahead 기반 Pure Pursuit 조향각."""
        curv      = self._curvature[near_idx]
        lookahead = self.LA_BASE + self.LA_K * self._cur_speed - self.LA_K_CURV * curv
        lookahead = max(0.4, lookahead)

        goal, _, _ = self._find_lookahead(pos, lookahead, yaw)
        if goal is None:
            return None

        dx =  goal[0] - pos[0];  dy = goal[1] - pos[1]
        lx =  np.cos(yaw) * dx + np.sin(yaw) * dy
        ly = -np.sin(yaw) * dx + np.cos(yaw) * dy
        L  = np.hypot(lx, ly)
        if L < 1e-4:
            return None

        return float(np.arctan2(2.0 * self.WHEELBASE * ly, L ** 2))

    # ── Stanley 조향각 계산 ────────────────────────────────────────────────
    def _steer_stanley(self, cte, heading_err):
        """
        steer = -(heading_err + arctan(K * cte / v))
        cte>0(좌) → 우회전(음), heading_err>0(좌) → 우회전(음)
        """
        v = max(self._cur_speed, 0.3)
        return float(-(heading_err + np.arctan(self.STANLEY_K * cte / v)))

    # ── LQR 조향각 계산 ───────────────────────────────────────────────────
    def _steer_lqr(self, cte, heading_err):
        """steer = -(k_cte * cte + k_heading * heading_err)"""
        if not self._lqr_available or len(self._lqr_speeds) == 0:
            return self._steer_stanley(cte, heading_err)

        v = float(np.clip(self._cur_speed,
                          self._lqr_speeds[0], self._lqr_speeds[-1]))
        k_cte     = float(np.interp(v, self._lqr_speeds, self._lqr_K[:, 0]))
        k_heading = float(np.interp(v, self._lqr_speeds, self._lqr_K[:, 1]))
        return float(-(k_cte * cte + k_heading * heading_err))

    # ── 오도메트리 콜백 ────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        if self._drive_state == 'REVERSING':
            self._handle_reversing()
            return

        if self._drive_state == 'FTG':
            if self._scan_data is not None:
                front_dist = self._front_min_dist(self._scan_data)
                if front_dist > self.OBSTACLE_TRIGGER_DIST * 2.0:
                    self.get_logger().info(
                        f'[FTG→NORMAL] 전방 clear {front_dist:.2f}m')
                    self._drive_state = 'NORMAL'
                    self._initialized = False
                    self._pos_history.clear()
                else:
                    self._follow_the_gap()
                    return
            else:
                self._follow_the_gap()
                return

        px  = msg.pose.pose.position.x
        py  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._cur_speed = float(np.hypot(vx, vy))

        lookahead_base = max(0.5, self.LA_BASE + self.LA_K * self._cur_speed)
        _, _, near_idx = self._find_lookahead(
            np.array([px, py]), lookahead_base, yaw)

        target_speed = float(self._v_profile[near_idx]) * self.SPEED_SCALE

        if self._scan_data is not None:
            front_dist = self._front_min_dist(self._scan_data)
            if front_dist < self.OBSTACLE_TRIGGER_DIST:
                self.get_logger().warn(
                    f'[장애물→FTG] 전방 {front_dist:.2f}m')
                self._drive_state = 'FTG'
                self._pos_history.clear()
                return
            if front_dist < self.BRAKE_START_DIST:
                brake_ratio = (front_dist - self.COLLISION_THRESH) / \
                              (self.BRAKE_START_DIST - self.COLLISION_THRESH)
                target_speed *= float(np.clip(brake_ratio, 0.0, 1.0))

        cte, heading_err = self._compute_cte_and_heading_err(px, py, yaw, near_idx)
        curv = self._curvature[near_idx]
        pos  = np.array([px, py])

        # ── 제어기 선택 ───────────────────────────────────────────────────
        if abs(cte) >= self.CTE_RECOVER_THRESH or curv >= self.HIGH_CURV:
            steer     = self._steer_app(pos, yaw, near_idx)
            ctrl_name = 'APP'
            if steer is None:
                steer = 0.0
        elif curv <= self.LOW_CURV:
            steer     = self._steer_lqr(cte, heading_err)
            ctrl_name = 'LQR'
        else:
            steer     = self._steer_stanley(cte, heading_err)
            ctrl_name = 'Stanley'

        # ── 경계 소프트 블렌딩 ────────────────────────────────────────────
        lo = self.HIGH_CURV - self.BLEND_MARGIN
        hi = self.HIGH_CURV + self.BLEND_MARGIN
        if lo < curv < hi:
            t   = (curv - lo) / (2 * self.BLEND_MARGIN)
            s_a = self._steer_app(pos, yaw, near_idx) or steer
            s_s = self._steer_stanley(cte, heading_err)
            steer     = (1.0 - t) * s_s + t * s_a
            ctrl_name = f'blend(Stanley↔APP t={t:.2f})'

        lo = self.LOW_CURV - self.BLEND_MARGIN
        hi = self.LOW_CURV + self.BLEND_MARGIN
        if lo < curv < hi:
            t   = (curv - lo) / (2 * self.BLEND_MARGIN)
            s_s = self._steer_stanley(cte, heading_err)
            s_l = self._steer_lqr(cte, heading_err)
            steer     = (1.0 - t) * s_l + t * s_s
            ctrl_name = f'blend(LQR↔Stanley t={t:.2f})'

        steer += self._side_bias_correction()

        stuck, stuck_disp = self._check_stuck(px, py)
        if stuck:
            self.get_logger().warn(
                f'[STUCK→FTG] {self.STUCK_TIME}s 변위 {stuck_disp:.3f}m')
            self._drive_state = 'FTG'
            self._pos_history.clear()
            return

        steer = float(np.clip(steer, -self.MAX_STEER, self.MAX_STEER))
        steer = self.STEER_ALPHA * self._steer_prev + (1.0 - self.STEER_ALPHA) * steer
        self._steer_prev = steer

        self.get_logger().info(
            f'[{ctrl_name}] κ={curv:.3f} CTE={cte:.3f}m '
            f'steer={np.degrees(steer):.1f}°  spd={target_speed:.2f}m/s')

        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = steer
        cmd.drive.speed          = float(target_speed)
        self.drive_pub.publish(cmd)

    # ── Adaptive Lookahead 포인트 탐색 (아크 길이 기반 보간) ──────────────
    def _find_lookahead(self, pos, lookahead, yaw=None):
        dists = np.linalg.norm(self.wp[:, :2] - pos, axis=1)
        n     = len(self.wp)
        search_range = 50

        if not self._initialized:
            near = int(np.argmin(dists))
            if yaw is not None:
                fwd    = np.array([np.cos(yaw), np.sin(yaw)])
                wp_dir = self.wp[(near + 1) % n, :2] - self.wp[near, :2]
                if np.dot(fwd, wp_dir) < 0:
                    self.get_logger().warn(
                        '웨이포인트 방향이 차량 헤딩과 반대 → 배열 역전')
                    self.wp              = self.wp[::-1]
                    self._curvature      = self._curvature[::-1]
                    self._heading        = self._heading[::-1]
                    self._v_profile      = self._v_profile[::-1]
                    near = int(n - 1 - near)
            self._initialized = True
        else:
            candidates = [(self._near_idx + i) % n for i in range(search_range)]
            near = candidates[int(np.argmin([dists[i] for i in candidates]))]

        self._near_idx = near

        arc = 0.0
        for i in range(n):
            i0  = (near + i)     % n
            i1  = (near + i + 1) % n
            seg = float(np.linalg.norm(self.wp[i1, :2] - self.wp[i0, :2]))
            if arc + seg >= lookahead:
                alpha    = float(np.clip((lookahead - arc) / max(seg, 1e-6), 0.0, 1.0))
                goal_xy  = (1.0 - alpha) * self.wp[i0, :2] + alpha * self.wp[i1, :2]
                goal_spd = (1.0 - alpha) * self.wp[i0, 2]  + alpha * self.wp[i1, 2]
                return goal_xy, goal_spd, near
            arc += seg

        return None, None, near


def main(args=None):
    rclpy.init(args=args)
    node = HybridController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
