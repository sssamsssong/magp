#!/usr/bin/env python3
"""
auto_localize.py — 기동 시 라이다 스캔을 맵에 맞춰 초기 위치를 자동으로 잡는다.

왜 필요한가
───────────
AMCL 은 `recovery_alpha_slow/fast: 0` (전역 복구 꺼짐)이라 초기 파티클을
`init_pose` 주변에만 뿌린다. 게다가 정지 상태에서는 `update_min_d`(2.5cm) /
`update_min_a`(1.7°) 조건 때문에 **필터를 갱신하지도, `/amcl_pose` 를 발행하지도
않는다.** 그래서 지금까지는 RViz 에서 `2D Pose Estimate` 를 사람이 찍어줘야
정렬이 됐다 — 대회 출발선에서 노트북 열고 클릭하는 건 현실적이지 않다.

이 노드가 그 클릭을 대신한다:

  1. 맵에서 "가장 가까운 장애물까지의 거리" 장(거리변환)을 한 번 만든다
  2. `init_pose` 주변으로 후보 자세(x, y, yaw)를 격자로 깔고
  3. 각 후보에서 현재 스캔 점들을 맵 좌표로 옮겨, 장애물까지 평균 거리를 잰다
  4. 가장 잘 맞는 자세를 `/initialpose` 로 발행한다

발행 즉시 AMCL 이 파티클을 그 자리에 다시 뿌리고 `/amcl_pose` 를 내보내므로,
컨트롤러도 pose 를 받고 출발 게이트의 "AMCL ✓" 도 뜬다.

★ 이건 전역 위치추정이 아니다. `init_pose` 근처에서만 찾는다 (기본 ±0.5m, ±15°).
  차를 출발선 근처에 놓기만 하면 되고, 센티미터 단위로 맞출 필요가 없어진다.
  아예 다른 곳에 놓으면 실패하며, 그때는 점수가 나쁘다고 경고한다.

  좌우 대칭이 강한 트랙에서는 뒤집힌 자세가 비슷한 점수를 낼 수 있다.
  탐색 범위를 좁게 두는 이유이기도 하다 (기본값이면 방향이 뒤집힐 여지가 없다).

재실행: `/race/relocalize` 에 Bool true 를 쏘면 다시 맞춘다.
"""
import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class AutoLocalize(Node):
    def __init__(self):
        super().__init__('auto_localize')

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        # 탐색 범위 — 차를 출발선 "근처"에 놓는 정도의 오차만 흡수한다.
        # 넓히면 대칭 구조에서 엉뚱한 자세로 수렴할 위험이 커진다.
        self.declare_parameter('search_xy', 0.5)          # ± m
        self.declare_parameter('search_yaw_deg', 15.0)    # ± deg
        self.declare_parameter('step_xy', 0.05)           # m (맵 해상도와 같게)
        self.declare_parameter('step_yaw_deg', 1.0)       # deg
        self.declare_parameter('max_range', 8.0)          # 이보다 먼 점은 안 씀
        self.declare_parameter('n_beams', 180)            # 스캔 다운샘플 개수
        self.declare_parameter('laser_offset_x', 0.27)    # base_link → laser
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('startup_delay', 2.0)      # 스캔 안정화 대기 (s)
        # 평균 오차가 이보다 크면 "제대로 못 맞췄다" 고 본다 (m)
        self.declare_parameter('fit_warn', 0.20)

        gp = lambda n: self.get_parameter(n).value
        self.init = (float(gp('init_x')), float(gp('init_y')), float(gp('init_yaw')))
        self.laser_dx = float(gp('laser_offset_x'))
        self.laser_dy = float(gp('laser_offset_y'))
        self.max_range = float(gp('max_range'))
        self.n_beams = int(gp('n_beams'))
        self.fit_warn = float(gp('fit_warn'))

        self.scan = None
        self.done = False
        self.dist = None          # 거리변환 (m 단위), 맵 픽셀 격자

        self._load_map(str(gp('map_yaml')))

        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         '/initialpose', 10)
        # 정합 결과(평균오차 m)를 알려 mission_manager 가 "출발 준비" 판정에 쓴다.
        # 늦게 뜨는 구독자도 받도록 래치.
        self.fit_pub = self.create_publisher(
            Float32, '/race/localized',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, str(gp('scan_topic')),
                                 self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/race/relocalize', self._redo_cb, 10)
        self.create_timer(float(gp('startup_delay')), self._try)

        self.get_logger().info(
            f'AutoLocalize 시작 — init ({self.init[0]:.2f}, {self.init[1]:.2f}, '
            f'{math.degrees(self.init[2]):.1f}°) 주변 ±{gp("search_xy")}m '
            f'±{gp("search_yaw_deg")}° 탐색')

    # ── 맵 → 거리변환 ──────────────────────────────────────────────────────
    def _load_map(self, path):
        if not path or not os.path.exists(path):
            self.get_logger().error(f'맵 yaml 을 못 찾음: {path}')
            return
        cfg = yaml.safe_load(open(path))
        self.res = float(cfg['resolution'])
        self.ox = float(cfg['origin'][0])
        self.oy = float(cfg['origin'][1])
        occ_th = float(cfg.get('occupied_thresh', 0.65))
        img_path = os.path.join(os.path.dirname(path), cfg['image'])
        img = np.asarray(Image.open(img_path).convert('L'))
        self.H, self.W = img.shape
        # map_server 규약: 밝을수록 자유공간. (255-v)/255 가 점유 확률
        occupied = ((255 - img.astype(np.float32)) / 255.0) > occ_th
        # 자유공간에서 "가장 가까운 장애물까지" 거리 (픽셀 → m)
        free = (~occupied).astype(np.uint8)
        self.dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        self.get_logger().info(
            f'맵 로드: {self.W}x{self.H}px @ {self.res}m, 장애물 {int(occupied.sum())}px')

    # ── 콜백 ───────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        self.scan = msg

    def _redo_cb(self, msg: Bool):
        if msg.data:
            self.done = False
            self.get_logger().info('재정렬 요청 수신')
            self._try()

    def _try(self):
        if self.done or self.dist is None:
            return
        if self.scan is None:
            self.get_logger().warn('스캔 대기 중…')
            return
        self.done = True
        self._align(self.scan)

    # ── 정렬 ───────────────────────────────────────────────────────────────
    def _align(self, scan: LaserScan):
        gp = lambda n: self.get_parameter(n).value

        # 스캔 → base_link 좌표의 점들 (유효한 것만, 균등 다운샘플)
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max,
                                                              self.max_range))
        if ok.sum() < 20:
            self.get_logger().error(f'쓸 만한 빔이 {int(ok.sum())}개뿐 — 정렬 중단')
            return
        idx = np.where(ok)[0]
        if len(idx) > self.n_beams:
            idx = idx[np.linspace(0, len(idx) - 1, self.n_beams).astype(int)]
        px = r[idx] * np.cos(a[idx]) + self.laser_dx
        py = r[idx] * np.sin(a[idx]) + self.laser_dy      # (N, ) base_link 기준

        # 후보 격자
        sxy = float(gp('search_xy')); step = float(gp('step_xy'))
        syaw = math.radians(float(gp('search_yaw_deg')))
        dyaw = math.radians(float(gp('step_yaw_deg')))
        dx = np.arange(-sxy, sxy + 1e-9, step)
        dy = np.arange(-sxy, sxy + 1e-9, step)
        dth = np.arange(-syaw, syaw + 1e-9, dyaw)
        x0, y0, th0 = self.init

        best = (np.inf, x0, y0, th0)
        # yaw 별로 한 번 회전시켜 놓고 xy 는 브로드캐스트로 훑는다
        for t in dth:
            th = th0 + t
            c, s = math.cos(th), math.sin(th)
            wx = c * px - s * py + x0        # (N,)
            wy = s * px + c * py + y0
            # (nx, ny, N)
            gx = wx[None, None, :] + dx[:, None, None]
            gy = wy[None, None, :] + dy[None, :, None]
            ix = ((gx - self.ox) / self.res).astype(np.int32)
            iy = (self.H - (gy - self.oy) / self.res).astype(np.int32)
            valid = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
            ixc = np.clip(ix, 0, self.W - 1)
            iyc = np.clip(iy, 0, self.H - 1)
            d = self.dist[iyc, ixc]
            # 맵 밖으로 나간 점은 크게 벌점 (맵 밖은 정보가 없는 게 아니라 틀린 것)
            d = np.where(valid, d, 3.0)
            score = d.mean(axis=2)           # (nx, ny)
            k = int(np.argmin(score))
            i, j = np.unravel_index(k, score.shape)
            if score[i, j] < best[0]:
                best = (float(score[i, j]), x0 + dx[i], y0 + dy[j], th)

        fit, bx, by, bth = best
        d0 = math.hypot(bx - x0, by - y0)
        self.get_logger().info(
            f'정렬 결과: ({bx:+.3f}, {by:+.3f}, {math.degrees(bth):+.1f}°)  '
            f'평균오차 {fit*100:.1f}cm  |  init 대비 {d0*100:.1f}cm '
            f'{math.degrees(bth - th0):+.1f}°')

        if fit > self.fit_warn:
            self.get_logger().warn(
                f'⚠ 평균오차 {fit*100:.1f}cm — 맵과 잘 안 맞습니다. '
                '차가 출발선 근처에 있는지, 맵이 맞는지 확인하세요. '
                '(그래도 init_pose 보다는 나으므로 발행은 합니다)')

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(bx)
        msg.pose.pose.position.y = float(by)
        msg.pose.pose.orientation.z = math.sin(bth * 0.5)
        msg.pose.pose.orientation.w = math.cos(bth * 0.5)
        # 탐색 격자 크기 정도의 불확실성을 남긴다 — 0 으로 두면 AMCL 이
        # 파티클을 한 점에 몰아 이후 보정이 둔해진다
        cov = [0.0] * 36
        cov[0] = cov[7] = (2.0 * float(gp('step_xy'))) ** 2
        cov[35] = (2.0 * math.radians(float(gp('step_yaw_deg')))) ** 2
        msg.pose.covariance = cov
        self.pub.publish(msg)
        self.fit_pub.publish(Float32(data=float(fit)))
        self.get_logger().info('/initialpose 발행 — AMCL 이 이 자리로 재시드합니다')


def main():
    rclpy.init()
    node = AutoLocalize()
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
