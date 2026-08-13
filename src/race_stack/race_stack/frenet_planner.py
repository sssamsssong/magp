#!/usr/bin/env python3
"""frenet_planner — Frenet 회피 오프셋을 발행하는 독립 노드.

무엇을 하나
──────────
레이싱라인을 기준선으로 후보들(점별 법선으로 d 만큼 민 것)을 만들고, 장애물
여유·트랙 이탈·복귀력·변화량으로 비용을 매겨 **최적 횡오프셋 d 하나**를 고른다.
로직은 `obstacle_avoid.ObstacleAvoidMixin` — Stanley 가 저속에서 잘 쓰던 그것을
그대로 쓴다 (복사 아님, 같은 코드).

    발행:  /frenet/offset   Float64   ← 계약. 컨트롤러가 소비한다
           /frenet/status   Float64   진단용 (0=평상, 1=회피중, 2=가능후보없음)
           /frenet_candidates          RViz 마커
           /obstacle                   RViz 장애물 점

왜 경로(Path)가 아니라 스칼라 d 인가
────────────────────────────────────
웨이포인트에는 **속도 프로파일이 실려 있다**. Path 로 보내면 속도를 따로 실어야
하고 컨트롤러가 원본과 다시 짝지어야 한다. d 만 넘기면 각 컨트롤러가
`wps + d·normal` 세 줄로 적용하고 속도가 그대로 유지된다.

★ 컨트롤러별 호라이즌 (이식 시 유일한 함정)
────────────────────────────────────────────
d 에 수렴하는 속도가 컨트롤러마다 다르다.

    Stanley  δ = ψ_e + atan(k·e/v)      전방주시 지연 없음 → 즉시 반응
    MAP/L1   a_lat = 2v²sin(η)/L1       2차계 ζ=0.707, ωn=√2·v/L1
             → 오버슈트는 4.3% 로 작지만 **상승거리 1.8·L1/√2 ≈ 1.15m** (L1=0.9m)

즉 MAP 은 오프셋을 다 옮기는 데 1.15m 를 달려야 한다. Stanley 기준으로 맞춰둔
obs_horizon_min(0.8m)을 그대로 쓰면 **다 피하기 전에 장애물에 닿는다.**
경험칙: **회피 호라이즌 ≥ 2 × L1**. race_script.yaml 의
`frenet.per_controller.<이름>` 에서 컨트롤러별로 덮어쓴다.

안전
────
컨트롤러 쪽에서 **타임아웃 폴백**을 반드시 걸 것 — 이 노드가 죽거나 늦으면
d=0(레이싱라인)으로 돌아가야 한다. 이 저장소의 /slip/steer_limit 과 같은 패턴이다.
"""
import math
import os
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray

from race_stack.obstacle_avoid import ObstacleAvoidMixin, _load_map

LATCHED_IN = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FrenetPlanner(ObstacleAvoidMixin, Node):
    def __init__(self):
        super().__init__('frenet_planner')
        d = self.declare_parameter

        d('waypoints_csv', '')
        d('map_yaml',      '')
        d('bound_dir',     '')
        d('scan_topic',    '/scan')
        d('odom_topic',    '/odom')
        d('rate_hz',       50.0)
        d('mode',          'q2')

        # 라이다 장착 위치 (base_link 기준)
        d('laser_dx', 0.27)
        d('laser_dy', 0.0)

        # ── 호라이즌 ─ ★ 컨트롤러에 맞춰 넣을 것 (위 docstring 참고) ──────
        d('obs_horizon_sec',   0.6)
        d('obs_horizon_min',   0.8)
        d('obs_horizon_max',   3.0)
        d('obs_corridor_half', 0.45)
        d('obs_release_scale', 1.3)

        # ── Frenet ────────────────────────────────────────────────────
        d('frenet_dmax',      0.6)
        d('frenet_dstep',     0.10)
        d('frenet_clearance', 0.35)
        d('frenet_clear_cap', 0.8)
        d('frenet_w_offset',  1.0)
        d('frenet_w_change',  0.5)
        d('frenet_w_clear',   2.0)
        d('frenet_bounds',    True)
        d('frenet_votes',     3)
        d('track_margin',     0.15)
        # ★ 통과 여유 하한 [m] — 차 반폭 + 안전여유. 이보다 좁으면 못 지나간다.
        #   옛 판정(마스크 all())이 레이싱라인 자신을 27% 프레임에서 탈락시켰다.
        #   0.22 면 두 트랙 모두 라인 탈락 0% (_frenet_select 주석 참고).
        d('frenet_min_room',  0.22)
        # 다른 쪽 통로가 이만큼 더 넓어야 갈아탄다 [m]. 좌우 진동 방지.
        d('frenet_gap_hyst',  0.10)
        # 컨트롤러의 기준선 램프 속도 [m/s]. 같아야 플래너가 보는 "현재 위치"가
        # 실제와 맞는다 (race_script.yaml frenet.shift_rate 와 동일 값).
        d('shift_rate',       2.0)

        # ── 레이캐스팅 (믹스인이 self.get_parameter 로 직접 읽는다) ────────
        #   기본값은 stanley_controller 와 동일하게 맞췄다. 갈라지면 같은 코드가
        #   컨트롤러에 따라 다르게 동작한다.
        d('raycast_obs',      True)
        d('raycast_fov_deg',  140.0)
        d('raycast_beam_deg', 0.25)
        d('raycast_deficit',  0.30)
        d('raycast_pose_unc', 0.25)
        # 검사 (B): 빔 끝점이 벽에서 이만큼 떨어지면 장애물. 0 = 끔.
        #   좁은 통로 한가운데 장애물을 잡는다 (_raycast_obstacle_pts 참고).
        d('raycast_free_gap', 0.30)
        d('use_bounds',       False)
        d('debug',            True)

        p = lambda n: self.get_parameter(n).value

        # ── 믹스인이 요구하는 속성들 ──────────────────────────────────
        self.mode      = str(p('mode'))
        self.laser_dx  = float(p('laser_dx'))
        self.laser_dy  = float(p('laser_dy'))
        self.obs_h_sec = float(p('obs_horizon_sec'))
        self.obs_h_min = float(p('obs_horizon_min'))
        self.obs_h_max = float(p('obs_horizon_max'))
        self.obs_corr_hw       = float(p('obs_corridor_half'))
        self.obs_release_scale = float(p('obs_release_scale'))
        self.track_margin = float(p('track_margin'))
        self.use_bounds   = bool(p('use_bounds'))

        self.fr_dmax   = float(p('frenet_dmax'))
        step           = max(float(p('frenet_dstep')), 1e-3)
        n_side         = int(round(self.fr_dmax / step))
        self.fr_grid   = np.arange(-n_side, n_side + 1) * step
        self.fr_clear  = float(p('frenet_clearance'))
        self.fr_clr_cap = float(p('frenet_clear_cap'))
        self.fr_w_off  = float(p('frenet_w_offset'))
        self.fr_w_chg  = float(p('frenet_w_change'))
        self.fr_w_clr  = float(p('frenet_w_clear'))
        self.fr_bounds = bool(p('frenet_bounds'))
        self.fr_min_room = float(p('frenet_min_room'))
        self.fr_gap_hyst = float(p('frenet_gap_hyst'))
        self._fr_costs = deque(maxlen=max(int(p('frenet_votes')), 1))
        self._fr_best_d = 0.0
        self._fr_gap_room = 0.0
        self._fr_status = 0.0
        self._shift_rate = max(float(p('shift_rate')), 1e-3)
        self._dt = 1.0 / max(float(p('rate_hz')), 1.0)
        self._shift_cur = 0.0        # 컨트롤러와 같은 램프로 _tick 에서 따라간다

        self.free_votes = deque(maxlen=max(int(p('frenet_votes')), 1))
        self.lane_free  = True
        self.on_track_pts = np.empty((0, 2))
        self.blocking_pts = []

        self.pos = None
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.speed = 0.0
        self.latest_scan = None

        self.map_arr = self.map_meta = None
        self.track_mask = None
        self.ray_occ = None
        self.inner_bound = self.outer_bound = None
        self._wp_ds = 0.1
        self.wps = None

        # ── 자원 로드 ─────────────────────────────────────────────────
        self._load_waypoints(str(p('waypoints_csv')))
        my = str(p('map_yaml'))
        if my and os.path.exists(my):
            self.map_arr, self.map_meta = _load_map(my)
            self._build_track_mask(self.track_margin)
            self._build_raycast_map()
            self.get_logger().info(f'맵 로드: {os.path.basename(my)}')
        else:
            self.get_logger().warn(
                f'맵을 못 읽었다 ({my!r}) — 트랙 이탈 검사와 레이캐스팅이 꺼진다. '
                f'후보 선택이 장애물 여유만 보게 되므로 트랙 밖으로 나갈 수 있다.')
            self.fr_bounds = False

        # ── ROS ───────────────────────────────────────────────────────
        self.off_pub = self.create_publisher(Float64, '/frenet/offset', 10)
        self.st_pub  = self.create_publisher(Float64, '/frenet/status', 10)
        self.fr_pub  = (self.create_publisher(MarkerArray, '/frenet_candidates', 1)
                        if bool(p('debug')) else None)
        self.obs_pub = (self.create_publisher(MarkerArray, '/obstacle', 1)
                        if bool(p('debug')) else None)

        self.create_subscription(LaserScan, str(p('scan_topic')), self._scan_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._pose_cb, 10)
        self.create_subscription(Odometry, str(p('odom_topic')), self._odom_cb, 10)
        self.create_timer(1.0 / max(float(p('rate_hz')), 1.0), self._tick)

        self.get_logger().info(
            f'FrenetPlanner ready — 후보 {len(self.fr_grid)}개 '
            f'(±{self.fr_dmax:.2f}m/{step:.2f}m), 여유 {self.fr_clear:.2f}m, '
            f'호라이즌 {self.obs_h_min:.1f}~{self.obs_h_max:.1f}m '
            f'({self.obs_h_sec:.1f}s)')

    # ── 입력 ─────────────────────────────────────────────────────────
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        self.pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _odom_cb(self, msg: Odometry):
        self.speed = float(msg.twist.twist.linear.x)
        self.yaw_rate = float(msg.twist.twist.angular.z)

    # ── 본체 ─────────────────────────────────────────────────────────
    def _tick(self):
        if self.pos is None or self.wps is None:
            return
        d = self._frenet_select(float(self.pos[0]), float(self.pos[1]), self.yaw)
        m = Float64();  m.data = float(d);  self.off_pub.publish(m)
        s = Float64();  s.data = float(self._fr_status);  self.st_pub.publish(s)

        # ★ 컨트롤러가 실제로 옮기고 있는 위치를 여기서도 따라간다.
        #   예전엔 _shift_cur 이 **영원히 0** 이었다. 그러면
        #     · 비용의 w_change·|d − 현재| 가 w_offset·|d| 와 똑같아져
        #       히스테리시스 항이 아무 일도 안 한다 (오프셋 벌점만 두 배)
        #     · "지금 어느 통로에 있나" 를 몰라 좌우 유지 판정이 불가능하다
        #   컨트롤러와 같은 램프 속도를 써야 실제 위치와 어긋나지 않는다.
        step = self._shift_rate * self._dt
        self._shift_cur += float(np.clip(d - self._shift_cur, -step, step))

        if self._fr_status > 0.0:
            self.get_logger().info(
                f'[frenet] d={d:+.2f}m  통로 여유 {self._fr_gap_room:.2f}m'
                + ('  ★막힘' if self._fr_status >= 2.0 else ''),
                throttle_duration_sec=0.5)

    def _load_waypoints(self, path: str):
        import csv as _csv
        if not path or not os.path.exists(path):
            self.get_logger().error(f'웨이포인트를 못 읽었다: {path!r} — 회피 불가')
            return
        rows = []
        with open(path) as f:
            for r in _csv.reader(f):
                if not r or r[0].lstrip().startswith('#'):
                    continue
                try:
                    rows.append([float(x) for x in r[:3]])
                except ValueError:
                    continue
        if len(rows) < 5:
            self.get_logger().error(f'웨이포인트가 너무 적다 ({len(rows)}개)')
            return
        self.wps = np.array(rows)
        ds = np.hypot(np.diff(self.wps[:, 0]), np.diff(self.wps[:, 1]))
        self._wp_ds = float(np.median(ds)) if len(ds) else 0.1
        self.get_logger().info(
            f'웨이포인트 {len(self.wps)}개, 간격 {self._wp_ds:.3f}m')


def main():
    rclpy.init()
    node = FrenetPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
