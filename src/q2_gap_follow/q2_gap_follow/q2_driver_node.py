"""
Q2 Hybrid Driver — Wall-Follow 백본 + 장애물 감지 시 Gap-Follow 모드 전환

설계 개요
---------
- 평상시: 벽추종 PID (안정적, 일정한 라인)
- 전방 감지창에 장애물(클러스터)이 잡히면 → Gap-Follow 모드 전환
- 장애물 통과 후 → 일정 시간 cooldown 두고 Wall-Follow 복귀
- Disparity Extender로 0.5m 폭 장애물도 안전 마진 확보

토픽
----
입력 : /scan (sensor_msgs/LaserScan)
출력 : /drive (ackermann_msgs/AckermannDriveStamped)
      /q2/mode (std_msgs/String)             — 현재 모드 시각화용
      /q2/obstacles (visualization_msgs/MarkerArray) — 감지된 장애물 RViz 표시
"""

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class Q2Driver(Node):
    """Q2 — 정적 장애물 2개 회피 + 3바퀴 완주 전용 드라이버"""

    # 모드
    MODE_WALL = 'WALL'
    MODE_GAP = 'GAP'

    def __init__(self):
        super().__init__('q2_driver')

        # ===== 파라미터 선언 =====
        # 공통
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('min_speed', 1.0)
        self.declare_parameter('max_steering', 0.4)

        # Wall-Follow PID
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('desired_distance', 1.0)
        self.declare_parameter('lookahead', 1.0)
        self.declare_parameter('alpha_angle', 45.0)
        self.declare_parameter('follow_side', 'right')

        # Gap-Follow
        self.declare_parameter('bubble_radius', 0.35)        # 안전 버블 반경 (m)
        self.declare_parameter('max_lidar_dist', 4.0)        # 최대 유효거리 (m)
        self.declare_parameter('disparity_threshold', 0.3)   # disparity 임계 (m)
        self.declare_parameter('gap_fov_deg', 180.0)         # 전방 사용 각도

        # 장애물 감지
        self.declare_parameter('obstacle_detect_fov_deg', 60.0)  # 전방 감지창
        self.declare_parameter('obstacle_detect_range', 3.0)     # 감지 거리
        self.declare_parameter('obstacle_min_pts', 3)            # 최소 포인트 수
        self.declare_parameter('obstacle_max_width', 0.8)        # 0.5m 장애물 + 마진
        self.declare_parameter('cluster_gap', 0.25)              # 클러스터링 간극
        self.declare_parameter('mode_switch_cooldown', 1.5)      # 모드 전환 쿨다운 (s)

        # 제어
        self.declare_parameter('control_rate', 20.0)

        # ===== 파라미터 로드 =====
        gp = self.get_parameter
        self.max_speed = gp('max_speed').value
        self.min_speed = gp('min_speed').value
        self.max_steering = gp('max_steering').value

        self.kp = gp('kp').value
        self.ki = gp('ki').value
        self.kd = gp('kd').value
        self.desired_distance = gp('desired_distance').value
        self.lookahead = gp('lookahead').value
        self.alpha = np.radians(gp('alpha_angle').value)
        self.follow_side = gp('follow_side').value

        self.bubble_radius = gp('bubble_radius').value
        self.max_lidar_dist = gp('max_lidar_dist').value
        self.disparity_threshold = gp('disparity_threshold').value
        self.gap_fov = np.radians(gp('gap_fov_deg').value)

        self.det_fov = np.radians(gp('obstacle_detect_fov_deg').value)
        self.det_range = gp('obstacle_detect_range').value
        self.det_min_pts = gp('obstacle_min_pts').value
        self.det_max_width = gp('obstacle_max_width').value
        self.cluster_gap = gp('cluster_gap').value
        self.cooldown = gp('mode_switch_cooldown').value

        self.min_dt = 1.0 / gp('control_rate').value

        # ===== 내부 상태 =====
        self.mode = self.MODE_WALL
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()
        self.last_obstacle_time = self.get_clock().now()
        self.detected_obstacles = []   # 시각화용 (각도, 거리, 폭)
        self.enabled = True

        # ===== Pub / Sub =====
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.enabled_sub = self.create_subscription(
            Bool, '/race/enabled', self.enabled_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.mode_pub = self.create_publisher(String, '/q2/mode', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/q2/obstacles', 10)

        self.get_logger().info(
            f'Q2 Driver 시작 — follow_side={self.follow_side}, '
            f'max_speed={self.max_speed}')

    def enabled_callback(self, msg: Bool):
        if self.enabled and not msg.data:
            self.publish_stop()
        self.enabled = msg.data

    def publish_stop(self):
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = 'base_link'
        drive.drive.speed = 0.0
        drive.drive.steering_angle = 0.0
        self.drive_pub.publish(drive)

    # ===================================================================
    # 메인 콜백
    # ===================================================================
    def scan_callback(self, msg: LaserScan):
        if not self.enabled:
            return
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt < self.min_dt:
            return
        if dt > 1.0:
            self.prev_time = now
            return
        self.prev_time = now

        # 1) LiDAR 정제
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, self.max_lidar_dist)
        ranges = np.clip(ranges, 0.0, self.max_lidar_dist + 1.0)

        # 2) 전방 감지창에서 장애물 클러스터링
        obstacles = self._detect_obstacles(ranges, msg)
        self.detected_obstacles = obstacles

        # 3) 모드 결정 (히스테리시스 + cooldown)
        time_since_obs = (now - self.last_obstacle_time).nanoseconds * 1e-9
        if len(obstacles) > 0:
            self.last_obstacle_time = now
            if self.mode != self.MODE_GAP:
                self.get_logger().info(
                    f'[모드 전환] WALL → GAP  (장애물 {len(obstacles)}개 감지)')
            self.mode = self.MODE_GAP
        elif self.mode == self.MODE_GAP and time_since_obs > self.cooldown:
            self.get_logger().info('[모드 전환] GAP → WALL  (장애물 없음)')
            self.mode = self.MODE_WALL
            # PID 상태 초기화 (모드 전환 시 적분 누적 폭주 방지)
            self.prev_error = 0.0
            self.integral = 0.0

        # 4) 모드별 제어
        if self.mode == self.MODE_WALL:
            steering, speed = self._wall_follow(msg, dt)
        else:
            steering, speed = self._gap_follow(ranges, msg)

        # 5) 퍼블리시
        drive = AckermannDriveStamped()
        drive.header.stamp = now.to_msg()
        drive.header.frame_id = 'base_link'
        drive.drive.steering_angle = float(np.clip(
            steering, -self.max_steering, self.max_steering))
        drive.drive.speed = float(np.clip(speed, self.min_speed, self.max_speed))
        self.drive_pub.publish(drive)

        # 모드 + 마커 시각화
        self.mode_pub.publish(String(data=self.mode))
        self._publish_markers(msg)

    # ===================================================================
    # 장애물 감지 (전방 감지창 내 클러스터링)
    # ===================================================================
    def _detect_obstacles(self, ranges, scan):
        """전방 감지창에서 0.5m 정도의 클러스터를 찾아 반환.

        Returns: list of (mid_angle_rad, distance_m, width_m)
        """
        # 전방 감지창 인덱스 범위
        half_fov = self.det_fov / 2.0
        # scan.angle_min ~ angle_max 안에서 -half_fov ~ +half_fov 추출
        idx_min = int((-half_fov - scan.angle_min) / scan.angle_increment)
        idx_max = int((+half_fov - scan.angle_min) / scan.angle_increment)
        idx_min = max(0, idx_min)
        idx_max = min(len(ranges) - 1, idx_max)

        window = ranges[idx_min:idx_max + 1]
        n = len(window)
        if n == 0:
            return []

        # 감지 거리 이내 점만 candidate
        candidate = window < self.det_range

        obstacles = []
        cs = None
        for i in range(n):
            r = window[i]
            if candidate[i]:
                if cs is None:
                    cs = i
                    prev_r = r
                # 인접 점과 거리 차이가 클러스터 간극 이하면 같은 클러스터
                if abs(r - prev_r) > self.cluster_gap and cs is not None:
                    # 끊김 — 직전까지를 한 클러스터로 마감
                    obstacles.append(self._cluster_info(
                        window, cs, i - 1, idx_min, scan))
                    cs = i
                prev_r = r
            else:
                if cs is not None:
                    obstacles.append(self._cluster_info(
                        window, cs, i - 1, idx_min, scan))
                    cs = None
        if cs is not None:
            obstacles.append(self._cluster_info(
                window, cs, n - 1, idx_min, scan))

        # 필터: 최소 점 수 + 폭 제한 (벽이 아닌 0.5m 장애물만)
        filtered = []
        for mid_ang, dist, width, pts in obstacles:
            if pts < self.det_min_pts:
                continue
            if width > self.det_max_width:
                # 너무 넓으면 벽일 가능성 — 무시
                continue
            filtered.append((mid_ang, dist, width))
        return filtered

    def _cluster_info(self, window, s, e, idx_offset, scan):
        """클러스터 (s..e) 인덱스로부터 중심각, 평균거리, 폭(추정) 반환."""
        seg = window[s:e + 1]
        valid = seg[seg < self.det_range]
        if len(valid) == 0:
            return (0.0, 999.0, 0.0, 0)
        mean_r = float(np.mean(valid))
        # 중심 인덱스 → 각도
        mid_idx_global = idx_offset + (s + e) // 2
        mid_ang = scan.angle_min + mid_idx_global * scan.angle_increment
        # 폭 ≈ 2 * mean_r * sin(angular_width / 2)
        ang_width = (e - s) * scan.angle_increment
        width = 2.0 * mean_r * np.sin(ang_width / 2.0)
        return (mid_ang, mean_r, width, len(valid))

    # ===================================================================
    # Wall-Follow 모드 (안정적 직선·곡선 주행)
    # ===================================================================
    def _wall_follow(self, scan: LaserScan, dt: float):
        if self.follow_side == 'right':
            angle_b = -np.pi / 2
            angle_a = angle_b + self.alpha
        else:
            angle_b = np.pi / 2
            angle_a = angle_b - self.alpha

        a = self._get_range_at(scan, angle_a)
        b = self._get_range_at(scan, angle_b)

        if np.isnan(a) or np.isnan(b):
            return 0.0, self.min_speed

        theta = np.arctan2(a * np.cos(self.alpha) - b, a * np.sin(self.alpha))
        dist = b * np.cos(theta)
        projected = dist + self.lookahead * np.sin(theta)
        error = self.desired_distance - projected

        if self.follow_side == 'left':
            error = -error  # 좌측 추종이면 부호 반전

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 1e-6 else 0.0
        self.prev_error = error
        steering = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 속도 (조향각 비례 감속)
        a_s = abs(steering)
        if a_s < np.radians(10):
            speed = self.max_speed
        elif a_s < np.radians(20):
            speed = self.max_speed * 0.7
        else:
            speed = self.max_speed * 0.45

        return steering, speed

    def _get_range_at(self, scan, angle):
        idx = int((angle - scan.angle_min) / scan.angle_increment)
        idx = max(0, min(idx, len(scan.ranges) - 1))
        r = float(scan.ranges[idx])
        if not np.isfinite(r) or r < scan.range_min or r > scan.range_max:
            return float('nan')
        return r

    # ===================================================================
    # Gap-Follow 모드 (Disparity Extender + Best Gap)
    # ===================================================================
    def _gap_follow(self, ranges, scan):
        # 전방 윈도우만 사용
        half = int(self.gap_fov / 2.0 / scan.angle_increment)
        c = len(ranges) // 2
        s_idx = max(0, c - half)
        e_idx = min(len(ranges), c + half)
        proc = ranges[s_idx:e_idx].copy()
        proc = np.clip(proc, 0.0, self.max_lidar_dist)

        # 1) Disparity Extender — 거리 급변 지점을 0으로 확장
        proc = self._extend_disparities(proc, scan.angle_increment)

        # 2) Safety Bubble — 가장 가까운 점 주변 차단
        if proc.min() < self.bubble_radius * 2.0:
            min_idx = int(np.argmin(proc))
            min_d = max(proc[min_idx], 0.05)
            bubble_ang = np.arctan2(self.bubble_radius, min_d)
            bubble_pts = max(5, int(bubble_ang / scan.angle_increment))
            lo = max(0, min_idx - bubble_pts)
            hi = min(len(proc), min_idx + bubble_pts + 1)
            proc[lo:hi] = 0.0

        # 3) 가장 긴 연속 비-제로 구간 찾기
        bs, be = self._best_gap(proc, threshold=0.1)
        if be <= bs:
            return 0.0, self.min_speed

        # 4) 갭 안에서 가장 먼 점을 타겟으로
        best_idx = bs + int(np.argmax(proc[bs:be + 1]))
        # 전방 윈도우 내부 인덱스 → 차량 기준 조향각
        target_idx_global = s_idx + best_idx
        target_angle = scan.angle_min + target_idx_global * scan.angle_increment

        steering = float(np.clip(target_angle, -self.max_steering, self.max_steering))

        # 속도: 회피 중에는 항상 보수적으로
        front_d = float(np.mean(proc[max(0, len(proc)//2 - 5): len(proc)//2 + 5]))
        a_s = abs(steering)
        if a_s < np.radians(10):
            speed = self.max_speed * 0.75
        elif a_s < np.radians(20):
            speed = self.max_speed * 0.55
        else:
            speed = self.max_speed * 0.35
        if front_d < 1.0:
            speed = min(speed, self.min_speed * 1.5)

        return steering, speed

    def _extend_disparities(self, ranges, angle_inc):
        """인접 점 거리 차가 큰 곳(disparity)에서, 가까운 쪽 거리로
        가까운 거리에서의 차폭만큼 0으로 확장."""
        out = ranges.copy()
        car_half_width = 0.18  # 차폭 절반 + 마진
        for i in range(1, len(out)):
            diff = out[i] - out[i - 1]
            if abs(diff) > self.disparity_threshold:
                if diff > 0:
                    # i-1쪽이 더 가까움 → i 방향으로 확장
                    closer_d = max(out[i - 1], 0.05)
                    n_ext = int(np.arctan2(car_half_width, closer_d) / angle_inc)
                    end = min(len(out), i + n_ext)
                    out[i:end] = np.minimum(out[i:end], out[i - 1])
                else:
                    closer_d = max(out[i], 0.05)
                    n_ext = int(np.arctan2(car_half_width, closer_d) / angle_inc)
                    start = max(0, i - n_ext)
                    out[start:i] = np.minimum(out[start:i], out[i])
        return out

    def _best_gap(self, ranges, threshold=0.1):
        mask = ranges > threshold
        bs = be = bl = 0
        cs = cl = 0
        ig = False
        for i, v in enumerate(mask):
            if v:
                if not ig:
                    cs, ig = i, True
                    cl = 1
                else:
                    cl += 1
                if cl > bl:
                    bl = cl
                    bs = cs
                    be = i
            else:
                ig = False
                cl = 0
        return bs, be

    # ===================================================================
    # 시각화 (RViz)
    # ===================================================================
    def _publish_markers(self, scan):
        ma = MarkerArray()
        # 항상 DELETEALL 먼저
        clr = Marker()
        clr.action = Marker.DELETEALL
        ma.markers.append(clr)

        for i, (ang, dist, width) in enumerate(self.detected_obstacles):
            m = Marker()
            m.header = scan.header
            m.header.frame_id = 'ego_racecar/laser'
            m.ns = 'q2_obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position = Point(
                x=float(dist * np.cos(ang)),
                y=float(dist * np.sin(ang)),
                z=0.1)
            m.pose.orientation.w = 1.0
            m.scale.x = max(0.2, float(width))
            m.scale.y = max(0.2, float(width))
            m.scale.z = 0.3
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.0, a=0.8)
            m.lifetime.sec = 0
            m.lifetime.nanosec = 200_000_000  # 200ms
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = Q2Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
