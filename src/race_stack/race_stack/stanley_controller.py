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
import time
from collections import deque

import cv2
import numpy as np
import scipy.spatial.distance
import yaml
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# ── 저장소 루트 (주행 종료 시 실주행 로그 저장용, gen_waypoints.py와 동일한 관례) ──
# 이 파일: <repo>/src/race_stack/race_stack/stanley_controller.py
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # <repo>/src/race_stack


# ★ /race/enabled 는 mission_manager 가 **래치(TRANSIENT_LOCAL)** 로 발행한다.
#   기본 QoS(VOLATILE)로 구독하면 구독자가 생기기 전에 나간 값을 못 받는다.
#   race.launch.py 는 컨트롤러를 5~8초 지연 기동하므로 항상 그 상황이고, 그래서
#   출발 게이트가 걸어둔 enabled=false 를 놓쳐 버튼도 안 눌렀는데 출발했다
#   (2026-08-04 실차). 래치로 구독해야 마지막 값을 받는다.
LATCHED_IN = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

_REPO_ROOT = (os.path.expanduser(os.environ['RACE_WS']) if os.environ.get('RACE_WS')
              else os.path.dirname(os.path.dirname(_PKG_DIR)))


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

    # ★ YAML 의 image: 를 그대로 따른다 (map_server 와 동일한 규약).
    #   예전엔 확장자를 ('.png', '.pgm') 순으로 추측했는데, maps/ 에는 같은 이름의
    #   미리보기 PNG 가 함께 있다. track_2nd_clean.png 는 1300x900 인데 실제 맵
    #   track_2nd_clean.pgm 은 170x154 다. PNG 를 집으면 map_meta 의 h/w 가
    #   6.7배 커져 _world_to_img 좌표가 통째로 어긋나고, 경계 폴리곤(픽셀 좌표)
    #   밖으로 떨어져 장애물이 단 하나도 감지되지 않는다. (2026-07-21 실차 확인)
    img_p = None
    img_name = cfg.get('image')
    if img_name:
        cand = os.path.join(os.path.dirname(os.path.abspath(map_yaml_path)),
                            img_name)
        if os.path.exists(cand):
            img_p = cand
    if img_p is None:                      # image: 키가 없는 옛 맵 폴백
        for ext in ('.pgm', '.png'):       # 순서도 pgm 우선으로
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
        # 구독할 오도메트리. 기본은 imu_odom 이 내는 자이로 기반 /odom_imu.
        # 원본으로 되돌리려면 race_script.yaml 에서 '/odom' 으로 바꾼다.
        self.declare_parameter('odom_topic',      '/odom_imu')
        # ★ 초기 pose 시드 (race_script.yaml 의 localization.init_*)
        #   AMCL 의 첫 /amcl_pose 를 받기 전까지 "차가 여기 있다" 고 가정한다.
        #   자세한 이유는 _odom_cb 의 시드 블록 주석 참조.
        self.declare_parameter('init_x',           0.0)
        self.declare_parameter('init_y',           0.0)
        self.declare_parameter('init_yaw',         0.0)
        self.declare_parameter('max_speed',        5.0)
        self.declare_parameter('min_speed',        1.0)
        self.declare_parameter('max_steer',        0.36)
        self.declare_parameter('wheelbase',        0.33)
        self.declare_parameter('stanley_gain',     0.8)
        self.declare_parameter('stanley_k',        1.8)
        self.declare_parameter('stanley_ks',       1.3)
        self.declare_parameter('vel_scale',        1.0)
        self.declare_parameter('obstacle_decel',   0.85)
        # ★ 실제 진로(예측 원호) 통로 반폭(m). 차 반폭 + 여유.
        #   웨이포인트 기준만 보면 "레이싱라인 위" 장애물만 잡힌다. 실주행 궤적은
        #   추종 오차 때문에 라인에서 벌어지므로(특히 코너 직후), 실제로 부딪힐
        #   위치에 놓인 장애물이 obs2lane 밖이라 통째로 무시됐다. (2026-07-21 실차)
        self.declare_parameter('obs_corridor_half_width', 0.30)
        # 회피 중 통로를 이 배로 넓혀 본다 (해제 조건 완화). 1.0 = 히스테리시스 없음.
        self.declare_parameter('obs_release_scale',       1.6)
        # ★ 회피량은 "장애물 폭 = 내 차 폭" 으로 가정해 계산한다.
        #   1:1 레이싱에서 장애물은 곧 상대 차량이다. 작은 콘에도 같은 기준을
        #   쓰면 넉넉하게 비껴가므로 안전하다.
        #     중심간 필요 간격 = 내 반폭 + 상대 반폭 = car_width
        #     여기에 여유를 더한 만큼 목표점을 옆으로 민다.
        self.declare_parameter('car_width',        0.30)   # 차 폭 (m)
        self.declare_parameter('obstacle_margin',  0.12)   # 스치지 않을 여유 (m)
        # 기준선을 옮기는 속도(m/s). 계단으로 옮기면 CTE 가 튀어 조향이 포화된다.
        self.declare_parameter('obstacle_shift_rate', 0.9)
        # 장애물이 "앞을 막았다"고 볼 전방 경로 구간 — 속도에 비례한다.
        #   horizon = clip(min + sec × speed, min, max)
        # 고정 3m 는 1.8m/s 에서 1.7초 앞까지 보는 셈이라, 아직 한참 남은 장애물에
        # 미리 반응해 라인을 벗어난다. 회피에 필요한 거리는 속도에 비례한다.
        self.declare_parameter('obs_horizon_sec',  0.3)
        self.declare_parameter('obs_horizon_min',  0.5)
        self.declare_parameter('obs_horizon_max',  3.0)
        # 차단 판정 투표 창(프레임). 과반이 차단이면 회피.
        # 라이다 40Hz 기준 5프레임 = 125ms = 1.8m/s 에서 0.23m 주행.
        # 예전 10프레임(250ms=0.45m)은 감시거리 1.04m 의 절반을 판정에 썼다.
        self.declare_parameter('obs_vote_frames',  5)

        # ── Frenet 후보 궤적 회피 (2026-08-04) ─────────────────────────
        # 기존 방식: "막혔다" → 래치한 한쪽으로 고정량(car_width+margin) 이동.
        #   장애물 폭·위치와 무관하게 항상 같은 양을 같은 쪽으로 민다. 트랙이
        #   좁아 그쪽이 더 위험해도, 살짝만 비켜도 되는 상황에도 똑같이 민다.
        # 새 방식: 레이싱라인을 기준선(s축)으로 삼고 횡오프셋 d 후보를 여러 개
        #   깔아, 각 후보 경로가 장애물·트랙경계와 얼마나 떨어지는지 실제로 재서
        #   비용이 최소인 d 를 고른다. 검출 파이프라인(통로 판정·투표)은 그대로다 —
        #   바뀌는 건 "얼마나 어느 쪽으로" 뿐이다.
        # ★ 래치가 필요 없어진다: 연속성은 w_change 비용이 만든다. 그래서 장애물
        #   두 개가 좌우로 있을 때 중간을 고르는 것 같은 판단이 자연히 나온다.
        self.declare_parameter('frenet_enabled',    True)   # False = 옛 래치 방식
        self.declare_parameter('frenet_candidates', 9)      # 홀수 권장 (0 포함)
        self.declare_parameter('frenet_d_max',      0.60)   # 최대 횡오프셋 (m)
        self.declare_parameter('frenet_clearance',  0.25)   # 후보 경로가 확보해야 할
                                                            # 장애물까지 최소 거리 (m)
        self.declare_parameter('frenet_w_offset',   1.0)    # |d| 비용 — 라인 복귀력
        self.declare_parameter('frenet_w_change',   0.8)    # |d-현재| 비용 — 히스테리시스
        # ★ w_clear 는 w_offset 보다 작아야 한다 (2026-08-04 실차에서 배움).
        #   옆으로 Δd 옮기면 여유도 대략 Δd 늘어난다. 그래서 w_clear > w_offset 이면
        #   비용 변화가 (w_offset - w_clear)·Δd < 0 — **갈수록 이득**이 되어
        #   clear_cap 에 닿을 때까지 최대한 벗어나는 게 최적해가 된다.
        #   실제로 정중앙 점 장애물 하나에 45cm 를 밀어 경로가 크게 흔들렸다.
        #   작게 두면 "실행 가능한 최소 오프셋"이 최적해가 된다 (같은 상황 30cm).
        self.declare_parameter('frenet_w_clear',    0.5)    # 여유거리 보상 가중
        self.declare_parameter('frenet_clear_cap',  0.35)   # 이 이상 여유는 더 안 쳐준다
        self.declare_parameter('frenet_bounds',     True)   # 트랙 경계 이탈 후보 배제
        self.declare_parameter('frenet_debug',      True)   # /frenet_candidates 마커
        # 라이다 장착 오프셋 (base_link → laser). 격자 점유 판정에 필수.
        # ★ f1tenth_stack/launch/bringup_launch.py 의 static_baselink_to_laser
        #   값과 반드시 일치해야 한다 (현재 x=0.27, y=0.0).
        self.declare_parameter('laser_offset_x',   0.27)
        self.declare_parameter('laser_offset_y',   0.0)
        self.declare_parameter('interp_scale',     20)
        self.declare_parameter('lookahead_min',    0.5)
        self.declare_parameter('lookahead_max',    2.0)
        self.declare_parameter('lookahead_gain',   7.0)
        self.declare_parameter('ff_gain',          1.0)
        self.declare_parameter('ff_preview_sec',   0.10)
        self.declare_parameter('steer_lpf',        1.0)

        # ── 벽 스침 회피 (wall avoidance) ──────────────────────────────
        # 2시 방향 벽이 다가와 우측 전면이 스칠 것 같으면 살짝 왼쪽으로 튼다.
        # AEB(정지)로는 못 막는, "부딪히진 않지만 스치는" 상황용.
        self.declare_parameter('wall_avoid_enabled',   False)
        self.declare_parameter('wall_avoid_half_width', 0.17)  # 차폭/2 (m)
        self.declare_parameter('wall_avoid_clearance',  0.18)  # 추가 여유 (m)
        self.declare_parameter('wall_avoid_lookahead',  1.2)   # 전방 감시 거리 (m)
        self.declare_parameter('wall_avoid_gain',       0.6)   # 침범량(m)→조향(rad)
        self.declare_parameter('wall_avoid_max',        0.10)  # 회피 조향 상한 (rad)
        self.declare_parameter('wall_avoid_lpf',        0.4)   # 회피 조향 저역통과
        self.declare_parameter('wall_avoid_min_range',  0.12)  # 차체 반사 제거 (m)
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
        # 이보다 벽에 가까운 반사는 '벽' 으로 본다 (m). 라인 위 최소 여유보다
        # 작아야 라인이 마스크 안에 남는다 (이 트랙 최소 0.36m)
        self.declare_parameter('track_margin',     0.28)
        # 경계 폴리곤을 겹쳐 쓸 것인가 — 트랙 밖 자유공간을 덜어낼 때만 유용
        self.declare_parameter('use_bounds',       False)
        # ── 빔별 기대거리 비교(레이캐스팅) ─────────────────────────────
        # 마스크 방식(track_margin)은 "이 반사점이 벽 근처인가"만 보므로,
        # AMCL 오차가 margin 을 넘으면 벽 반사가 그대로 장애물이 된다.
        # 레이캐스팅은 "이 빔이 맵 벽보다 뚜렷하게 앞에서 끊겼는가"를 본다.
        self.declare_parameter('raycast_obs',       True)
        self.declare_parameter('raycast_pose_unc',  0.25)   # 위치추정 불확실성(m)
        self.declare_parameter('raycast_deficit',   0.30)   # 기대보다 이만큼 짧아야 장애물
        self.declare_parameter('raycast_fov_deg',   100.0)  # 전방 감시 화각(±)
        self.declare_parameter('raycast_beam_deg',  1.0)    # 빔 샘플 간격

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
        self.obs_corr_hw = p('obs_corridor_half_width')
        self.obs_release_scale = p('obs_release_scale')
        self.car_width  = p('car_width')
        self.obs_margin = p('obstacle_margin')
        # 장애물을 내 차 폭으로 보고: 중심간 car_width + 여유
        self.obs_shift  = self.car_width + self.obs_margin
        self.obs_shift_rate = p('obstacle_shift_rate')
        self.fr_on      = bool(p('frenet_enabled'))
        self.fr_n       = max(3, int(p('frenet_candidates')))
        self.fr_dmax    = float(p('frenet_d_max'))
        # ★ 요구 여유는 검출 통로 반폭보다 작으면 안 된다.
        #   검출은 obs_corridor_half_width 로 "막혔다" 고 판정하는데 Frenet 이
        #   그보다 좁은 기준으로 "라인 그대로도 괜찮다" 고 답하면, 막혔다고 해놓고
        #   아무 회피도 안 하는 상태로 굳는다 (두 층이 다른 기준을 보는 문제).
        self.fr_clear   = float(p('frenet_clearance'))
        if self.fr_clear < self.obs_corr_hw:
            self.get_logger().warn(
                f'frenet.clearance({self.fr_clear:.2f}) < '
                f'obs_corridor_half_width({self.obs_corr_hw:.2f}) — '
                f'검출 기준에 맞춰 {self.obs_corr_hw:.2f} 로 올립니다')
            self.fr_clear = self.obs_corr_hw
        self.fr_w_off   = float(p('frenet_w_offset'))
        self.fr_w_chg   = float(p('frenet_w_change'))
        self.fr_w_clr   = float(p('frenet_w_clear'))
        self.fr_clr_cap = float(p('frenet_clear_cap'))
        self.fr_bounds  = bool(p('frenet_bounds'))
        self.fr_debug   = bool(p('frenet_debug'))
        # 후보 오프셋 격자 — 0 이 정확히 포함되도록 홀수개를 권장한다
        self.fr_grid    = np.linspace(-self.fr_dmax, self.fr_dmax, self.fr_n)
        self.track_margin = float(p('track_margin'))
        self.use_bounds = bool(p('use_bounds'))
        self.laser_dx   = p('laser_offset_x')
        self.laser_dy   = p('laser_offset_y')
        self.obs_h_sec  = p('obs_horizon_sec')
        self.obs_h_min  = p('obs_horizon_min')
        self.obs_h_max  = p('obs_horizon_max')
        self.interp     = int(p('interp_scale'))
        self.L_min      = p('lookahead_min')
        self.L_max      = p('lookahead_max')
        self.L_gain     = p('lookahead_gain')
        self.wheelbase  = p('wheelbase')
        self.ff_gain    = p('ff_gain')
        self.ff_prev_s  = p('ff_preview_sec')
        self.steer_lpf  = p('steer_lpf')

        self.wa_on      = bool(p('wall_avoid_enabled'))
        self.wa_hw      = p('wall_avoid_half_width')
        self.wa_clear   = p('wall_avoid_clearance')
        self.wa_look    = p('wall_avoid_lookahead')
        self.wa_gain    = p('wall_avoid_gain')
        self.wa_max     = p('wall_avoid_max')
        self.wa_lpf     = p('wall_avoid_lpf')
        self.wa_rmin    = p('wall_avoid_min_range')
        self.latest_scan = None       # 벽 회피용 최신 스캔
        self._wa_prev   = 0.0         # 회피 조향 LPF 상태
        self._wa_warned = 0.0         # 로그 스로틀용 시각

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

        # ── 곡률 사전계산 (피드포워드용, 로드 시 1회) ──────────────────
        self.kappa  = None
        self._wp_ds = 0.1
        if self.wps is not None and len(self.wps) > 10:
            self.kappa, self._wp_ds = self._precompute_curvature()
            self.get_logger().info(
                f'curvature precomputed: |κ|max={np.abs(self.kappa).max():.2f} '
                f'(spacing {self._wp_ds:.3f}m)')

        # ── state ─────────────────────────────────────────────────────
        self.enabled = False   # ★ mission_manager 의 /race/enabled(래치)를 받아야 켜진다.
        #   True 로 두면 그 값이 도착하기 전 한 틱이라도 먼저 나가버린다.
        #   mission_manager 가 없는 단독 실행이면 "정지 중: /race/enabled=false"
        #   로그가 이유를 알려준다 — 조용히 달려나가는 것보다 낫다.
        self.pos            = None
        self.yaw            = 0.0
        self.speed          = 0.0
        self.lane_free      = True
        self.yaw_rate       = 0.0
        self._shift_side    = 0      # 회피 방향 래치 (+1 좌 / -1 우 / 0 없음)
        self._shift_cur     = 0.0    # 현재 적용 중인 기준선 이동량 (램프)
        self._shift_vec     = None
        # 마지막 N 프레임 투표만 쓰므로 deque 로 자동 폐기.
        # (list 로 두면 스캔마다 append 되어 40Hz × 주행시간 만큼 무한히 쌓였다)
        self.free_votes: deque[bool] = deque(
            maxlen=max(1, int(p('obs_vote_frames'))))
        # "전방 경로를 막고 있는" 장애물 점들. 회피 방향 계산과 마커 표시에 쓴다.
        self.blocking_pts: list[np.ndarray] = []
        # 트랙 안에 있는 모든 장애물 점 (막고 있든 아니든). Frenet 후보 평가는
        # "지금 진로를 막았나" 가 아니라 "저 후보로 가면 뭐가 있나" 를 봐야 하므로
        # blocking_pts 로는 부족하다 — 옆으로 피한 자리의 장애물이 안 들어온다.
        self.on_track_pts: np.ndarray = np.zeros((0, 2))
        self._fr_costs: deque = deque(          # 후보별 비용의 이동평균 창
            maxlen=max(1, int(p('obs_vote_frames'))))
        self._fr_best_d = 0.0                   # 마지막으로 고른 오프셋 (로그용)

        # ── pose 융합 상태 ─────────────────────────────────────────────
        # /amcl_pose 는 느리고(이동해야 갱신) /odom 은 빠르지만 드리프트.
        # amcl 이 올 때마다 map←odom 보정(th,tx,ty)을 갱신하고, 제어 주기마다
        # 최신 odom 에 보정을 적용해 "지금" 위치를 50Hz 로 추정한다.
        self._odom_buf      = deque(maxlen=200)   # (t, x, y, yaw) 최근 ~2s
        self._odom_last     = None                # 최신 odom 샘플
        self._amcl_pose     = None                # 원시 amcl (odom 폴백용)
        self._map_from_odom = None                # (th, tx, ty) 보정 변환
        self._steer_prev    = 0.0                 # 조향 LPF 상태
        self._amcl_seen     = False               # /amcl_pose 를 한 번이라도 받았나
        self._seed_pose     = (float(p('init_x')), float(p('init_y')),
                               float(p('init_yaw')))
        self._seed_warn_t   = None                # AMCL 미수신 경고 스로틀

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

            self._build_track_mask(float(p('bound_ignore_px')))
            self._build_raycast_map()

        # ── subscriptions / publishers ─────────────────────────────────
        self.create_subscription(Bool,
            '/race/enabled', self._enabled_cb, LATCHED_IN)
        self.create_subscription(PoseWithCovarianceStamped,
            '/amcl_pose', self._pose_cb, 10)
        # ★ 기본이 /odom_imu 다 (2026-08-04). 원본 /odom 의 각속도는 서보 "명령"
        #   으로 계산돼(ω=v·tan(δ)/L) 서보 지연·타이어 슬립만큼 과대평가된다.
        #   이 yaw_rate 가 진로 원호 예측(_in_predicted_corridor)에 그대로 쓰이므로
        #   장애물 판정까지 같이 틀어졌다. imu_odom 이 자이로 실측으로 바꿔 준다.
        self.create_subscription(Odometry,
            p('odom_topic'), self._odom_cb, 10)
        # Q2 는 장애물 격자 판정에, 그 외 모드는 벽 회피에 스캔이 필요하다.
        if self.mode == 'q2' or self.wa_on:
            self.create_subscription(LaserScan,
                '/scan', self._scan_cb, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive',    10)
        self.wp_pub    = self.create_publisher(Marker,                 '/waypoint', 10)
        self.obs_pub   = self.create_publisher(MarkerArray,            '/obstacle', 10)
        self.fr_pub    = (self.create_publisher(MarkerArray, '/frenet_candidates', 5)
                          if self.fr_debug else None)
        # ── RViz 시각화: 전체 레이싱라인(속도 컬러) + 차 이동궤적 ─────────
        # 노트북 RViz 가 늦게 접속해도 받도록 1Hz 로 주기 재발행한다.
        self.path_pub    = self.create_publisher(Marker, '/waypoints_path', 10)
        self.carpath_pub = self.create_publisher(Path,   '/car_path',       10)
        self._car_path = Path()
        self._car_path.header.frame_id = 'map'
        self._last_trail = None
        # 실주행 경로 로그 — RViz 표시용 self._car_path와 별개로, 주행 종료 시
        # logs/ 에 CSV로 저장해 게인 튜닝 비교에 쓴다 (save_run_log 참조).
        self._trail_log = []
        self._run_t0 = self.get_clock().now()
        self.create_timer(1.0, self._publish_racing_line)   # 1Hz 재발행

        # ── 고정 주기 제어 루프 (50Hz) ─────────────────────────────────
        # 매 주기 (amcl 보정 + 최신 odom)으로 "지금" pose 를 추정해 조향을
        # 새로 계산·발행한다. pose 콜백 주기(이동 거리 조건)에 묶이지 않으므로
        # - mux timeout(0.2s) 데드락 없음 (정지 시에도 0속도 발행 = odom kick)
        # - 고속에서도 수십 cm 지난 위치가 아닌 현재 위치 기준으로 조향
        self.create_timer(0.02, self._cmd_tick)

        self.get_logger().info(f'StanleyController ready [mode={self.mode}]')

    def _cmd_tick(self):
        pose = self._current_pose()
        if not self.enabled or self.wps is None or pose is None:
            self._publish_cmd(0.0, 0.0)
            # 왜 멈춰 있는지 로그로 남긴다 — "로그는 정상인데 차가 안 움직임" 을
            # 다시 겪지 않도록. 세 가지 원인이 겉으로는 똑같이 보인다.
            why = ('/race/enabled=false (mission_manager 가 껐음)' if not self.enabled
                   else '웨이포인트 없음' if self.wps is None
                   else 'pose 없음 — odom 미수신 (imu_odom 이 떠 있는지 확인)')
            self.get_logger().warn(f'정지 중: {why}', throttle_duration_sec=3.0)
            return
        # AMCL 이 계속 조용하면 시드 가정만으로 달리고 있다는 뜻이다.
        if not self._amcl_seen:
            self.get_logger().warn(
                'AMCL pose 미수신 — init_pose 가정으로 주행 중입니다. '
                '실제 위치와 다르면 라인을 벗어납니다 (RViz 에서 스캔-맵 정합 확인)',
                throttle_duration_sec=5.0)
        self.pos = np.array([pose[0], pose[1]])   # scan_cb 등 공용 상태 갱신
        self.yaw = pose[2]
        steer, speed = self._compute_cmd(*pose)
        self._publish_cmd(steer, speed)
        self._update_car_trail(pose[0], pose[1])

    def _current_pose(self):
        """amcl 보정 ⊕ 최신 odom = 고주기 pose 추정. 없으면 amcl 원시값 폴백."""
        if self._map_from_odom is not None and self._odom_last is not None:
            th, tx, ty = self._map_from_odom
            _, ox, oy, oyaw = self._odom_last
            c, s = math.cos(th), math.sin(th)
            return (tx + c * ox - s * oy,
                    ty + s * ox + c * oy,
                    _normalize(th + oyaw))
        return self._amcl_pose

    # ── event callbacks ────────────────────────────────────────────────────

    def _enabled_cb(self, msg: Bool):
        if self.enabled and not msg.data:
            self._steer_prev = 0.0   # 재활성화 시 이전 조향 재사용 방지
            self._publish_cmd(0.0, 0.0)
        self.enabled = msg.data

    def _odom_cb(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z   # 진로 예측(원호) 용
        p = msg.pose.pose
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        entry = (t, p.position.x, p.position.y, _yaw(p.orientation))
        self._odom_last = entry
        self._odom_buf.append(entry)

        # ── 초기 pose 시드 ────────────────────────────────────────────
        # ★ 왜 필요한가 (2026-08-04 실차: 로그는 정상인데 차가 안 움직임)
        #   예전엔 /amcl_pose 를 한 번이라도 받아야만 주행이 시작됐다.
        #   그런데 AMCL 이 /amcl_pose 를 내는 시점은
        #     (1) 첫 스캔을 받았을 때 1회,
        #     (2) 그 뒤로는 update_min_d(2.5cm)/update_min_a(1.7도) 만큼 움직였을 때
        #   뿐이다. race.launch.py 는 lifecycle 을 5초, 컨트롤러를 8초로 지연
        #   기동하므로 (1) 이 컨트롤러가 뜨기 전에 끝나 있다. 래치 토픽도 아니라
        #   놓치면 끝이고, 차가 멈춰 있으니 (2) 도 영원히 안 온다 →
        #   "pose 가 없어 0속도 발행 → 안 움직이니 pose 도 안 옴" 데드락.
        #
        #   그래서 첫 odom 이 오는 순간 **init_pose 에 있다고 가정**하고 보정을
        #   시드한다. AMCL 도 같은 값으로 파티클을 뿌리므로 둘의 가정이 일치하고,
        #   진짜 /amcl_pose 가 오면 _pose_cb 가 즉시 덮어쓴다.
        #   기다림에 주행을 걸지 않는 것이 핵심이다.
        if self._map_from_odom is None and not self._amcl_seen:
            sx, sy, syaw = self._seed_pose
            _, ox, oy, oyaw = entry
            th = _normalize(syaw - oyaw)
            c, s = math.cos(th), math.sin(th)
            self._map_from_odom = (th, sx - (c * ox - s * oy),
                                   sy - (s * ox + c * oy))
            self.get_logger().info(
                f'초기 pose 시드: ({sx:.3f}, {sy:.3f}, {syaw:.3f}rad) — '
                'AMCL 첫 수신 전까지 이 가정으로 주행합니다')

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        """amcl 수신 → map←odom 보정 변환 갱신 (제어는 _cmd_tick 이 50Hz로)."""
        px  = msg.pose.pose.position.x
        py  = msg.pose.pose.position.y
        yaw = _yaw(msg.pose.pose.orientation)
        self._amcl_pose = (px, py, yaw)
        if not self._amcl_seen:
            self._amcl_seen = True
            sx, sy, _ = self._seed_pose
            self.get_logger().info(
                f'AMCL pose 첫 수신 ({px:.3f}, {py:.3f}) — 시드 가정 해제. '
                f'시드와의 차이 {math.hypot(px - sx, py - sy):.2f}m')
            self._report_start_alignment(px, py, yaw)

        # amcl 측정 시각과 가장 가까운 odom 샘플 기준으로 보정 계산
        # (amcl 은 그 시각의 pose 이므로, 그 시각의 odom 과 짝지어야 정확)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        od = (min(self._odom_buf, key=lambda e: abs(e[0] - t))
              if self._odom_buf else self._odom_last)
        if od is None:
            return
        _, ox, oy, oyaw = od
        th = _normalize(yaw - oyaw)
        c, s = math.cos(th), math.sin(th)
        self._map_from_odom = (th, px - (c * ox - s * oy), py - (s * ox + c * oy))

    def _report_start_alignment(self, px: float, py: float, yaw: float):
        """출발 자세가 레이싱라인과 얼마나 어긋났는지 알린다.

        ★ 왜 (2026-08-04 실차)
          "출발이 불안정하다" 의 원인은 위치추정이 아니라 **차를 비스듬히 놓은
          것**이었다. auto_localize 는 ±15° 를 탐색해 놓인 각도를 정확히 찾아내고,
          Stanley 는 출발하자마자 그 오차를 잡느라 크게 꺾는다. 실제 6회 주행의
          출발 방향이 -6° ~ -25° 로 흩어져 있었다.
          숫자로 보여주면 **버튼을 누르기 전에** 차를 바로 놓을 수 있다.
        """
        if self.wps is None or len(self.wps) < 3:
            return
        n = len(self.wps)
        i = int(np.argmin(np.hypot(self.wps[:, 0] - px, self.wps[:, 1] - py)))
        tang = self.wps[(i + 1) % n, :2] - self.wps[(i - 1) % n, :2]
        path_yaw = math.atan2(tang[1], tang[0])
        dyaw = math.degrees(_normalize(yaw - path_yaw))
        tn = float(np.hypot(*tang))
        left = (np.array([-tang[1], tang[0]]) / tn if tn > 1e-6
                else np.array([-math.sin(yaw), math.cos(yaw)]))
        cte = float((np.array([px, py]) - self.wps[i, :2]) @ left)

        msg = (f'출발 정렬: 라인과 방향차 {dyaw:+.1f}°, 횡오차 {cte*100:+.0f}cm '
               f'(최근접 wp {i})')
        if abs(dyaw) > 10.0 or abs(cte) > 0.25:
            self.get_logger().warn(
                msg + ' — 출발 직후 크게 꺾습니다. 차를 라인 방향으로 다시 놓는 것을 권합니다')
        else:
            self.get_logger().info(msg + ' — 양호')

    # ── main control loop (50Hz, _cmd_tick 에서 호출) ──────────────────────

    def _compute_cmd(self, px: float, py: float, yaw: float):
        speed = abs(self.speed)

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
        # ★ 회피 중에는 기준선(CTE 를 재는 대상)도 같이 옮긴다.
        #   목표점만 밀고 closest 를 원래 라인에 두면, CTE 항이 "라인에서
        #   벗어났다"며 되당겨 회피 조향을 상쇄한다. 그래서 회피가 둔했다.
        #   회피 중 우리가 추종하는 경로는 "라인을 옆으로 민 것" 이므로
        #   기준선도 같이 밀어야 Stanley 가 저항 없이 그 쪽으로 조향한다.
        closest = wps[nearest, :2]
        if self.mode == 'q2' and getattr(self, '_shift_vec', None) is not None:
            closest = closest + self._shift_vec

        # path_yaw = 경로의 "국소 접선" 방향 (교과서 Stanley).
        # 이전엔 최근접점→lookahead 목표의 현(chord)을 썼는데, 곡선에서 현은
        # 항상 코너 안쪽을 가리켜 정상상태 오프셋 tan(편향각)×(v+ks)/k 만큼
        # 안쪽으로 파고든다 (이 트랙 기준 0.3~0.6m → 가운데 장애물 충돌 원인).
        # 접선은 중심차분(±0.1m)으로 계산 — 전방차분은 편향이 남는다.
        m_t = max(1, int(round(0.1 / self._wp_ds)))
        j_f = (nearest + m_t) % n
        j_b = (nearest - m_t) % n
        path_yaw = math.atan2(wps[j_f, 1] - wps[j_b, 1],
                              wps[j_f, 0] - wps[j_b, 0])

        # Q2 회피 중에는 시프트된 목표점을 향해 조향 (회피 기동 유지)
        if self.mode == 'q2' and not self.lane_free:
            path_yaw = math.atan2(target[1] - py, target[0] - px)
        cte_size = float(np.hypot(px - closest[0], py - closest[1]))
        cte_sign = float(np.sign(np.cross(
            [math.cos(path_yaw), math.sin(path_yaw)],
            [px - closest[0],    py - closest[1]])))
        cte     = cte_size * cte_sign
        yaw_err = _normalize(path_yaw - yaw)

        # ── 곡률 피드포워드: 코너의 "정답 조향각"을 미리 적용 ──────────
        # Stanley 피드백은 오차가 생겨야 반응하지만, 피드포워드는 경로 곡률
        # 만으로 조향을 선행한다. ff_preview_sec 은 서보 지연 보상용 예견.
        steer_ff = 0.0
        if self.kappa is not None:
            prev_m = int(speed * self.ff_prev_s / self._wp_ds)
            kap    = float(self.kappa[(nearest + prev_m) % n])
            steer_ff = self.ff_gain * math.atan(self.wheelbase * kap)

        v_safe = max(speed, 0.5)   # avoid division by near-zero
        steer  = steer_ff + self.gain * (yaw_err
                              - math.atan2(self.k_cte * cte, v_safe + self.ks))

        # ── 벽 스침 회피 보정 ──────────────────────────────────────────
        # Stanley 의 CTE 항이 이 보정에 저항한다(레이싱라인에서 밀려나므로).
        # 그래서 보정은 폭주하지 않고 유한한 횡변위에서 평형을 이룬다:
        #   0.1rad 보정 ≈ 12cm 횡변위 (gain1.5/k1.8/v1.8/ks1.5 기준).
        # 이 자기제한이 안전장치라, 일부러 없애지 말 것.
        steer += self._wall_avoid_steer()

        steer  = float(np.clip(steer, -self.max_steer, self.max_steer))

        # ── 조향 저역통과 (steer_lpf=1.0 이면 비활성) ───────────────────
        steer = self.steer_lpf * steer + (1.0 - self.steer_lpf) * self._steer_prev
        self._steer_prev = steer

        # ── target speed from CSV ──────────────────────────────────────
        cmd_v = float(wps[nearest, 2]) * self.vel_scale
        if self.mode == 'q2' and not self.lane_free:
            cmd_v *= self.obs_decel
        cmd_v = float(np.clip(cmd_v, self.min_speed, self.max_speed))

        self._pub_wp_marker(target)
        return steer, cmd_v

    def _precompute_curvature(self, span: float = 0.2, smooth: int = 5):
        """각 웨이포인트의 부호 있는 곡률(좌회전 +)을 로드 시 1회 계산.

        ±span(m) 떨어진 세 점의 Menger curvature 를 쓰고 이동평균으로
        스무딩한다. 반환: (kappa 배열, 평균 점 간격 ds)
        """
        w  = self.wps[:, :2]
        ds = float(np.mean(np.hypot(*(np.roll(w, -1, axis=0) - w).T)))
        m  = max(1, int(round(span / max(ds, 1e-6))))
        a  = np.roll(w,  m, axis=0)
        c  = np.roll(w, -m, axis=0)
        ab, bc, ac = w - a, c - w, c - a
        cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
        denom = (np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1)
                 * np.linalg.norm(ac, axis=1))
        kappa = 2.0 * cross / np.maximum(denom, 1e-9)
        kern  = np.ones(smooth) / smooth
        pad   = np.concatenate([kappa[-smooth:], kappa, kappa[:smooth]])
        kappa = np.convolve(pad, kern, 'same')[smooth:-smooth]
        return kappa, ds

    # ── Q2 obstacle avoidance ──────────────────────────────────────────────

    def _obstacle_shift(self, target: np.ndarray,
                        px: float, py: float, yaw: float) -> np.ndarray:
        # 창이 다 찼을 때만 판정 (과반). 히스테리시스로 깜빡임을 막는다.
        if len(self.free_votes) == self.free_votes.maxlen:
            was_free = self.lane_free
            self.lane_free = sum(self.free_votes) * 2 >= self.free_votes.maxlen
            # 상태가 바뀔 때만 찍는다 — 회피가 발동했는지/언제 풀렸는지 판별용.
            # "감지는 됐는데 반응이 약했나" vs "아예 못 봤나" 를 이걸로 가른다.
            if was_free != self.lane_free:
                if self.lane_free:
                    self.get_logger().info('[obstacle] 경로 확보 — 회피 해제')
                else:
                    # 차량 좌표계 위치까지 찍는다. 진짜 장애물인지(전방 0.5~1m,
                    # 횡 0 근처) 차 근처 노이즈인지 이걸로 갈린다.
                    if self.blocking_pts:
                        b = np.asarray(self.blocking_pts)
                        dx, dy = b[:, 0] - px, b[:, 1] - py
                        c, s = math.cos(-yaw), math.sin(-yaw)
                        fx = c * dx - s * dy          # 전방(+)
                        fy = s * dx + c * dy          # 좌측(+)
                        i = int(np.argmin(np.hypot(fx, fy)))
                        info = (f'최근접 전방 {fx[i]:+.2f}m 횡 {fy[i]:+.2f}m, '
                                f'전방범위 {fx.min():+.2f}~{fx.max():+.2f}m')
                    else:
                        info = '(점 없음)'
                    self.get_logger().warn(
                        f'[obstacle] 경로 차단! 점 {len(self.blocking_pts)}개, '
                        f'{info}, horizon {self._obs_horizon():.2f}m '
                        f'→ 목표점 {self.obs_shift}m 이동')

        # ── 회피 기준 방향은 "경로 법선" 으로 잡는다 ────────────────────
        # 차량 헤딩 법선을 쓰면 회피 조향으로 차가 돌 때마다 시프트 방향까지
        # 같이 돌아 되먹임이 생긴다. 경로 법선은 map 상에서 안정적이다.
        n_wp = len(self.wps)
        near = int(np.argmin(np.hypot(self.wps[:, 0] - px, self.wps[:, 1] - py)))
        m_t = max(1, int(round(0.1 / self._wp_ds)))
        tf = self.wps[(near + m_t) % n_wp, :2]
        tb = self.wps[(near - m_t) % n_wp, :2]
        tang = tf - tb
        tn = float(np.hypot(*tang))
        left = (np.array([-tang[1], tang[0]]) / tn if tn > 1e-6
                else np.array([-math.sin(yaw), math.cos(yaw)]))

        if not self.lane_free and self.blocking_pts:
            if self.fr_on:
                # ── Frenet 후보 궤적 ───────────────────────────────────
                # 래치가 없다. 연속성은 |d-현재| 비용이 만들고, 그래서 장애물이
                # 좌우로 둘일 때 "가운데" 같은 답도 자연히 나온다.
                goal = self._frenet_select(px, py, yaw)
            else:
                # ── 옛 방식: 방향 래치 + 고정량 ────────────────────────
                # 한 번 "왼쪽으로 피한다" 고 정하면 그 장애물을 지나칠 때까지 유지.
                # 매 프레임 다시 정하면, 피하느라 차가 돌면서 장애물이 반대편으로
                # 보여 부호가 뒤집히고 좌우로 크게 휘청인다.
                if self._shift_side == 0:
                    obs_c = np.mean(self.blocking_pts, axis=0)
                    lateral = float((obs_c - np.array([px, py])) @ left)
                    self._shift_side = -1.0 if lateral > 0 else 1.0
                goal = self._shift_side * self.obs_shift
        else:
            self._shift_side = 0
            self._fr_costs.clear()
            goal = 0.0

        # ── 램프 ───────────────────────────────────────────────────────
        # 기준선을 계단으로 0.42m 옮기면 CTE 가 그만큼 튀어 조향이 22.6도로
        # 포화된다(max_steer 20.6도). 해제 때는 반대 방향으로 같은 킥이 난다.
        # 이 왕복이 "휘청휘청" 의 정체다. 속도 제한을 걸어 부드럽게 옮긴다.
        step = self.obs_shift_rate / 50.0        # _cmd_tick 이 50Hz
        self._shift_cur += float(np.clip(goal - self._shift_cur, -step, step))

        if abs(self._shift_cur) > 1e-3:
            self._shift_vec = self._shift_cur * left
            target = target + self._shift_vec
        else:
            self._shift_vec = None
        return target

    # ── Frenet 후보 궤적 평가 ──────────────────────────────────────────────

    def _frenet_paths(self, px: float, py: float):
        """레이싱라인을 기준선으로 한 후보 경로들을 만든다.

        반환: (cands (N, M, 2) map 좌표, seg (M,2), left (M,2))
              후보가 만들어지지 않으면 (None, None, None)

        기준선은 "현재 위치에서 앞으로 horizon 만큼의 레이싱라인" 이고, 후보는
        그것을 각 점의 **법선 방향으로** d 만큼 민 것이다. 직선 평행이동이 아니라
        점마다 법선을 쓰기 때문에 코너에서도 라인의 곡률을 그대로 따라간다.
        """
        wps = self.wps
        if wps is None or len(wps) < 5:
            return None, None, None
        n = len(wps)
        near = int(np.argmin(np.hypot(wps[:, 0] - px, wps[:, 1] - py)))
        horizon = self._obs_horizon()
        m = max(3, int(round(horizon / max(self._wp_ds, 1e-3))))
        m = min(m, n - 1)
        idx = (near + np.arange(m + 1)) % n
        seg = wps[idx, :2]

        # 점별 접선 → 법선 (중심차분). 끝점은 한쪽 차분으로 대체된다.
        fwd = wps[(idx + 1) % n, :2]
        bwd = wps[(idx - 1) % n, :2]
        tang = fwd - bwd
        tn = np.hypot(tang[:, 0], tang[:, 1])
        ok = tn > 1e-9
        tang[ok] /= tn[ok, None]
        left = np.stack([-tang[:, 1], tang[:, 0]], axis=1)

        # (N, M, 2) — 후보 × 경로점 × 좌표
        cands = seg[None, :, :] + self.fr_grid[:, None, None] * left[None, :, :]
        return cands, seg, left

    def _frenet_select(self, px: float, py: float, yaw: float) -> float:
        """후보 오프셋 중 비용이 최소인 것을 고른다 (m, 좌측 +).

        비용 = w_offset·|d|            라인에서 멀어질수록 손해 (복귀력)
             + w_change·|d - 현재|     급하게 바꾸지 않도록 (히스테리시스)
             - w_clear·min(여유, cap)  장애물에서 멀수록 이득 (cap 이상은 무의미)
        여유 < frenet_clearance 이거나 트랙 밖으로 나가는 후보는 **실행 불가**.

        ★ 비용 벡터를 투표 창만큼 이동평균한다. 라이다 한 프레임의 노이즈로
          목표 오프셋이 튀면 그게 곧 조향 킥이 된다 (검출 쪽 free_votes 와 같은 이유).
        """
        cands, seg, left = self._frenet_paths(px, py)
        if cands is None:
            return 0.0
        N, M, _ = cands.shape
        flat = cands.reshape(N * M, 2)

        # ── 장애물까지의 여유 ──────────────────────────────────────────
        obs = self.on_track_pts
        if len(obs) == 0:
            obs = np.asarray(self.blocking_pts) if self.blocking_pts else obs
        if len(obs):
            # 관련 없는 먼 점은 버린다 — cdist 크기를 실제로 지배하는 건 이쪽이다
            near_m = (np.hypot(obs[:, 0] - px, obs[:, 1] - py)
                      < self._obs_horizon() + self.fr_dmax + 1.0)
            obs = obs[near_m]
        if len(obs):
            d_obs = scipy.spatial.distance.cdist(flat, obs).min(axis=1)
            clear = d_obs.reshape(N, M).min(axis=1)          # 후보별 최소 여유
        else:
            clear = np.full(N, np.inf)

        # ── 트랙 경계 이탈 검사 ────────────────────────────────────────
        inside = np.ones(N, bool)
        if self.fr_bounds and getattr(self, 'track_mask', None) is not None:
            h, w = self.map_meta[0], self.map_meta[1]
            res, ox, oy = self.map_meta[2], self.map_meta[3], self.map_meta[4]
            img = _world_to_img(flat, h, res, ox, oy)
            ix, iy = img[:, 0], img[:, 1]
            ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            good = np.zeros(N * M, bool)
            good[ok] = self.track_mask[iy[ok], ix[ok]]
            inside = good.reshape(N, M).all(axis=1)

        # ── 비용 ───────────────────────────────────────────────────────
        d = self.fr_grid
        cost = (self.fr_w_off * np.abs(d)
                + self.fr_w_chg * np.abs(d - self._shift_cur)
                - self.fr_w_clr * np.minimum(clear, self.fr_clr_cap))
        feasible = inside & (clear >= self.fr_clear)

        self._fr_costs.append(cost)
        cost_avg = np.mean(np.stack(self._fr_costs), axis=0)

        if feasible.any():
            masked = np.where(feasible, cost_avg, np.inf)
            best = int(np.argmin(masked))
        else:
            # 전부 막혔다 — 그나마 가장 여유 있는 쪽으로. 여기까지 오면 속도를
            # 줄이는 게 정답이지만 그건 AEB/감속의 영역이고, 조향은 최선을 다한다.
            best = int(np.argmax(np.where(inside, clear, -np.inf))
                       if inside.any() else np.argmax(clear))
            self.get_logger().warn(
                f'[frenet] 실행 가능한 후보 없음 — 최대여유 {clear[best]:.2f}m '
                f'(요구 {self.fr_clear:.2f}m), d={d[best]:+.2f}m',
                throttle_duration_sec=1.0)

        self._fr_best_d = float(d[best])
        if self.fr_pub is not None:
            self._pub_frenet(cands, feasible, best)
        return self._fr_best_d

    def _pub_frenet(self, cands, feasible, best):
        """후보 경로를 RViz 에 그린다 — 초록=선택, 파랑=가능, 빨강=불가."""
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i in range(len(cands)):
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.header.stamp = now
            mk.ns = 'frenet'
            mk.id = i
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.scale.x = 0.03 if i == best else 0.015
            if i == best:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.0, 1.0, 0.0, 1.0
            elif feasible[i]:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.2, 0.5, 1.0, 0.5
            else:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.2, 0.2, 0.4
            for x, y in cands[i]:
                pt = Point(); pt.x = float(x); pt.y = float(y); pt.z = 0.02
                mk.points.append(pt)
            arr.markers.append(mk)
        self.fr_pub.publish(arr)

    def _build_track_mask(self, ign: float):
        """경계 안쪽 판정을 픽셀 마스크로 사전계산 (기동 시 1회).

        ★ 왜 필요한가 — 측정으로 드러난 병목
          예전엔 스캔마다 점유 셀 하나하나에 cv2.pointPolygonTest 를 2번씩
          파이썬 루프로 돌렸다. 실제 트랙(벽 포함) 스캔에서 점유 셀이 2505개라
          호출이 5010회/스캔, _scan_cb 가 44.5ms(22.5Hz) 걸렸다. 라이다는 40Hz
          이므로 따라가지 못하고, 10프레임 투표에 445ms = 0.80m 주행이 걸렸다.
          감시거리가 1.04m 인데 판정에 0.80m 를 쓰면 피할 거리가 남지 않는다.

          거리변환으로 마스크를 한 번 만들어 두면 판정이 배열 인덱싱 한 번이 된다.
          (io > ign) ⟺ 외곽 폴리곤 내부에서 경계까지 거리 > ign
          (ii < -ign) ⟺ 내곽 폴리곤 외부에서 경계까지 거리 > ign
        """
        self.track_mask = None
        if self.map_meta is None:
            return
        h, w = self.map_meta[0], self.map_meta[1]
        res = self.map_meta[2]

        # ★ 맵에서 직접 만든다 (2026-08-05). 예전엔 inner/outer 폴리곤을 썼는데
        #   그건 사람이 클릭해 만든 저해상도 도형이다. half_clean 은 33m 트랙에
        #   inner 10점 / outer 28점뿐이라 실제 벽선을 전혀 못 따라갔다:
        #     · 마스크가 **실제로는 벽인 181px 를 트랙 안쪽으로 포함** →
        #       그 벽에 찍힌 라이다 점이 장애물로 오인됐다
        #     · 반대로 진짜 주행 가능한 456px 는 누락
        #   실차에서 장애물 없이도 벽을 피하려 들며 트랙 좌우로 요동쳤다.
        #   맵의 자유공간을 margin 만큼 침식하면 5cm 해상도로 실제 벽을 따라간다.
        #
        #   margin 은 "이보다 벽에 가까운 반사는 벽으로 본다" 는 뜻이다.
        #   라인 위 최소 여유(이 트랙 36cm)보다 작아야 라인이 마스크 안에 남는다.
        margin = float(self.track_margin)
        # map_arr 은 상하 반전(아래가 0행)이라 _world_to_img 규약(위가 0행)에 맞춘다
        occ_top = np.flipud(self.map_arr[:, :, 0]) > 0.5
        free = (~occ_top).astype(np.uint8)
        dist_m = cv2.distanceTransform(free, cv2.DIST_L2, 5) * res
        self.track_mask = dist_m > margin

        src = '맵 자유공간'
        # 경계 폴리곤이 있으면 "트랙 밖 자유공간"(피트·관중석 등)을 덜어내는
        # 용도로만 겹쳐 쓴다. 폴리곤이 엉성해도 벽을 장애물로 만들지는 않는다.
        if (self.use_bounds and self.inner_bound is not None
                and self.outer_bound is not None):
            outer_m = np.zeros((h, w), np.uint8)
            inner_m = np.zeros((h, w), np.uint8)
            cv2.fillPoly(outer_m, [self.outer_bound.astype(np.int32)], 1)
            cv2.fillPoly(inner_m, [self.inner_bound.astype(np.int32)], 1)
            self.track_mask &= (outer_m > 0) & (inner_m == 0)
            src += ' ∩ 경계 폴리곤'

        self.get_logger().info(
            f'track mask built: {int(self.track_mask.sum())} / {h*w} px '
            f'({src}, margin {margin:.2f}m)')

    # ── 빔별 기대거리 비교 (레이캐스팅) ────────────────────────────────────

    def _build_raycast_map(self):
        """레이캐스팅용 점유격자를 기동 시 1회 만든다.

        ★ 왜 맵을 부풀리나 (dilate)
          기대거리를 "정확한 위치" 기준으로 계산하면, AMCL 이 20cm 틀린 순간
          벽 반사가 기대보다 20cm 짧게 보여 장애물이 된다. 마스크 방식이 실패한
          것과 똑같은 이유다.

          그래서 맵의 벽을 위치추정 불확실성 반경만큼 부풀린 뒤 레이캐스팅한다.
          그러면 기대거리는 "불확실성 안의 어떤 위치에서든 가능한 **가장 짧은**
          거리" 가 된다. 실측이 그보다도 더 짧다면, 그 반사는 어떤 위치를
          가정해도 맵 벽으로 설명할 수 없다 — 즉 진짜 장애물이다.

          이 방향의 대가는 "벽에 바짝 붙은 장애물을 놓친다" 는 것이다. 반대
          방향의 오류(벽을 장애물로 봄)는 직선 9m 를 42cm 밀려 달리게 만들었으니,
          이쪽 대가가 훨씬 싸다.
        """
        self.ray_occ = None
        if self.map_meta is None or not bool(self.get_parameter('raycast_obs').value):
            return
        res = self.map_meta[2]
        unc = float(self.get_parameter('raycast_pose_unc').value)
        occ_top = (np.flipud(self.map_arr[:, :, 0]) > 0.5).astype(np.uint8)
        r_px = max(1, int(round(unc / res)))
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_px + 1, 2 * r_px + 1))
        self.ray_occ = cv2.dilate(occ_top, k) > 0
        self.get_logger().info(
            f'raycast map built: 벽을 {unc:.2f}m({r_px}px) 부풀림, '
            f'점유 {int(occ_top.sum())} → {int(self.ray_occ.sum())} px')

    def _raycast_expected(self, lx0: float, ly0: float,
                          angs: np.ndarray, rmax: float) -> np.ndarray:
        """(lx0,ly0) 에서 angs 방향으로 부풀린 맵을 쏴 기대거리를 구한다.

        벽에 닿지 않으면 rmax 를 돌려준다. 전 빔을 한 번에 행진시켜(B×S 배열)
        파이썬 루프를 없앴다 — _scan_cb 는 40Hz 안에 끝나야 한다.
        """
        h, w, res, ox, oy = self.map_meta
        step = res * 0.5
        t = np.arange(0.0, rmax + step, step)                 # (S,)
        xs = lx0 + np.cos(angs)[:, None] * t[None, :]
        ys = ly0 + np.sin(angs)[:, None] * t[None, :]
        ix = ((xs - ox) / res).astype(np.int32)
        iy = (h - (ys - oy) / res).astype(np.int32)
        inb = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        hit = np.zeros(ix.shape, bool)
        hit[inb] = self.ray_occ[iy[inb], ix[inb]]
        # 맵 밖으로 나간 지점은 더 볼 것이 없으므로 벽으로 간주해 행진을 멈춘다
        hit |= ~inb

        # ★ 출발점이 부풀린 벽 안에 있는 경우 (차가 벽에서 pose_unc 이내)
        #   그대로 두면 t=0 에서 즉시 "명중" 이라 기대거리가 전부 0 이 되고,
        #   장애물 탐지가 통째로 멎는다. 트랙 여유가 36cm 인데 부풀림이 25cm 라
        #   실제로 자주 일어난다. 광선이 일단 자유공간으로 빠져나온 뒤부터
        #   명중을 센다.
        free = ~hit
        escaped = free.any(axis=1)
        first_free = np.argmax(free, axis=1)
        after = np.arange(hit.shape[1])[None, :] > first_free[:, None]
        hit = hit & after

        any_hit = hit.any(axis=1)
        first = np.argmax(hit, axis=1)
        exp = np.where(any_hit, t[first], rmax)
        # 끝까지 자유공간을 못 만난 빔은 판단 근거가 없다 — 0 을 돌려주면
        # (meas < 0 - deficit) 이 항상 거짓이 되어 장애물로 오인하지 않는다.
        return np.where(escaped, exp, 0.0)

    def _raycast_obstacle_pts(self, msg: LaserScan) -> np.ndarray:
        """기대보다 뚜렷하게 짧게 끊긴 빔만 골라 map 좌표 점으로 돌려준다."""
        deficit = float(self.get_parameter('raycast_deficit').value)
        fov     = math.radians(float(self.get_parameter('raycast_fov_deg').value))
        bstep   = math.radians(float(self.get_parameter('raycast_beam_deg').value))

        n = len(msg.ranges)
        stride = max(1, int(round(bstep / max(msg.angle_increment, 1e-6))))
        idx = np.arange(0, n, stride)
        a_car = msg.angle_min + idx * msg.angle_increment       # 차량 기준 빔각
        keep = np.abs(a_car) <= fov * 0.5
        idx, a_car = idx[keep], a_car[keep]
        if len(idx) == 0:
            return np.zeros((0, 2))

        meas = np.asarray(msg.ranges, dtype=float)[idx]
        valid = np.isfinite(meas) & (meas > msg.range_min) & (meas < msg.range_max)

        horizon = self._obs_horizon()
        rmax = horizon + deficit + 0.5

        # 라이다 원점은 base_link 보다 앞에 있다 — 기대거리도 거기서 재야 한다
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        lx0 = float(self.pos[0]) + cy * self.laser_dx - sy * self.laser_dy
        ly0 = float(self.pos[1]) + sy * self.laser_dx + cy * self.laser_dy
        exp = self._raycast_expected(lx0, ly0, self.yaw + a_car, rmax)

        obs = valid & (meas < horizon) & (meas < exp - deficit)
        if not obs.any():
            return np.zeros((0, 2))
        r, a = meas[obs], (self.yaw + a_car)[obs]
        return np.vstack((lx0 + r * np.cos(a), ly0 + r * np.sin(a))).T

    def _predicted_track_path(self, horizon: float) -> np.ndarray:
        """앞으로 지나갈 경로를 "레이싱라인 + 현재 횡오차"로 예측 (map 좌표).

        ★ 왜 원호만으로는 부족한가
          원호 예측은 "지금 조향을 유지한다" 가정이라 코너 전후에서 실제 경로와
          어긋난다. 진입 직전엔 yaw rate 가 아직 안 붙어 직선에 가깝고, 탈출
          직후엔 아직 커서 과하게 안쪽으로 휜다. 그 사이 코너 안쪽 장애물을 놓친다.

          이 차는 Stanley 로 라인을 추종하므로 미래 경로는 "라인을 현재 횡오차만큼
          평행이동한 것" 에 가깝다. 라인의 곡률을 그대로 따르니 코너 기하가 자동으로
          반영된다. 라인을 판정 기준으로 쓰는 게 아니라 내 진로를 예측하는 데 쓴다.
        """
        wps = self.wps
        if wps is None or len(wps) < 3:
            return np.zeros((0, 2))
        n = len(wps)
        px, py = float(self.pos[0]), float(self.pos[1])
        near = int(np.argmin(np.hypot(wps[:, 0] - px, wps[:, 1] - py)))

        m = max(1, int(round(horizon / max(self._wp_ds, 1e-3))))
        m = min(m, n - 1)
        idx = (near + np.arange(m + 1)) % n
        seg = wps[idx, :2]

        # 현재 횡오차(부호 포함)를 라인 법선 방향으로 잰다
        j_f = (near + 1) % n
        tang = wps[j_f, :2] - wps[near, :2]
        nt = float(np.hypot(*tang))
        if nt < 1e-6:
            return seg
        tang = tang / nt
        left = np.array([-tang[1], tang[0]])
        cte = float((np.array([px, py]) - wps[near, :2]) @ left)

        return seg + left * cte           # 라인을 내 오차만큼 평행이동

    def _in_predicted_corridor(self, pts_world: np.ndarray) -> np.ndarray:
        """차가 실제로 지나갈 통로 안에 있는 점인가 (bool 배열).

        두 예측의 합집합이다 — 어느 하나만으로는 구간에 따라 빗나간다:
          (1) 순간 원호  : /odom yaw rate 로 R=v/w. 지금 조향 기준.
          (2) 라인+횡오차: 코너 기하를 따르는 예측 (_predicted_track_path).
        """
        if len(pts_world) == 0:
            return np.zeros(0, bool)

        d = pts_world - self.pos
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        fx = c * d[:, 0] - s * d[:, 1]        # 전방(+)
        fy = s * d[:, 0] + c * d[:, 1]        # 좌측(+)

        # ★ 슈미트 트리거 — 회피 중에는 통로를 넓혀서 본다.
        #   좁은 통로만 보면 회피 조향을 시작하는 순간 장애물이 통로 밖으로
        #   나가 감지가 풀리고, 라인으로 복귀하다 다시 감지되는 진동이 생긴다.
        #   해제 조건을 진입보다 느슨하게 두어 장애물을 지나칠 때까지 붙잡는다.
        scale = 1.0 if self.lane_free else self.obs_release_scale
        hw = self.obs_corr_hw * scale
        horizon = self._obs_horizon() * scale
        v, w = abs(self.speed), self.yaw_rate

        R = None
        if abs(w) > 1e-3 and v > 1e-3:
            R = v / w if self.speed >= 0 else -v / w
            if abs(R) > 1e3:
                R = None

        # (1) 순간 원호 통로
        if R is None:                          # 직진
            arc_in = (fx > 0.0) & (fx < horizon) & (np.abs(fy) < hw)
        else:
            dist = np.hypot(fx, fy - R)
            lateral = np.abs(dist - abs(R)) < hw
            sr = math.copysign(1.0, R)
            phi = np.mod(np.arctan2(sr * fx, sr * (R - fy)), 2.0 * math.pi)
            arc_in = lateral & (abs(R) * phi < horizon)

        # (2) 라인+횡오차 예측 경로 통로 — 코너 기하를 따라간다
        path = self._predicted_track_path(horizon)
        if len(path) == 0:
            return arc_in
        # 뒤쪽 점은 제외 (이미 지나친 장애물에 반응하지 않도록)
        ahead = fx > -0.05
        path_in = np.zeros(len(pts_world), bool)
        if ahead.any():
            dmin = scipy.spatial.distance.cdist(
                path, pts_world[ahead]).min(axis=0)
            path_in[ahead] = dmin < hw

        return arc_in | path_in

    def _obs_horizon(self) -> float:
        """속도에 비례하는 전방 감시 거리(m).

        회피에 필요한 거리는 속도에 비례한다. 고정값(구 3m)은 1.8m/s 에서
        1.7초 앞까지 보는 셈이라 아직 한참 남은 장애물에 미리 반응했다.
        저속에서도 최소 obs_horizon_min 은 확보한다 (0 이면 아무것도 못 본다).
        """
        return float(np.clip(self.obs_h_min + self.obs_h_sec * abs(self.speed),
                             self.obs_h_min, self.obs_h_max))

    def _wall_avoid_steer(self) -> float:
        """다가오는 벽면에서 멀어지는 방향의 보정 조향(rad). +는 좌회전.

        예: 2시 방향 벽이 우측 전면으로 다가옴 → 오른쪽 침범량 > 0 → 왼쪽으로 보정.

        핵심은 "가장 가까운 점"이 아니라 **침범량(encroachment)** 을 본다는 것.
        나란한 벽은 원하는 여유(half_width+clearance) 밖에 있으면 침범량 0 이라
        아무 보정도 안 나온다. 양쪽이 동시에 좁아지면(좁은 통로) 서로 상쇄돼
        가운데를 유지하고, 조향 대신 AEB 가 담당한다.
        """
        scan = self.latest_scan
        if not self.wa_on or scan is None:
            return 0.0

        ranges = np.asarray(scan.ranges, dtype=float)
        n = len(ranges)
        if n == 0:
            return 0.0
        ang = scan.angle_min + np.arange(n) * scan.angle_increment

        ok = np.isfinite(ranges) & (ranges > self.wa_rmin) & \
             (ranges < self.wa_look * 2.0)
        if not ok.any():
            return self._wa_smooth(0.0)
        r, a = ranges[ok], ang[ok]
        x, y = r * np.cos(a), r * np.sin(a)

        # 전방 감시 창 안의 점만
        fwd = (x > 0.0) & (x < self.wa_look)
        if not fwd.any():
            return self._wa_smooth(0.0)
        x, y = x[fwd], y[fwd]

        # 원하는 측면 여유를 얼마나 파고들었는가
        want = self.wa_hw + self.wa_clear
        enc = want - np.abs(y)
        near = enc > 0.0
        if not near.any():
            return self._wa_smooth(0.0)

        # ★ 전방거리로 가중하지 않는다.
        #   차는 감시창(1.2m ≈ 0.7초)을 곧 통과하므로 창 안의 모든 x 가 임박한
        #   상황이고, |y| 자체가 "그 지점에서의 측면 여유"다. 즉 창 안 최소 여유가
        #   그대로 구속조건이다. 예전엔 (1-x/look) 로 가중했는데, 수렴하는 벽은
        #   위험 지점이 앞쪽이라 정작 그곳을 0 으로 눌러 보정이 안 나왔다.
        score = enc

        right = float(score[near & (y < 0)].max()) if (near & (y < 0)).any() else 0.0
        left = float(score[near & (y > 0)].max()) if (near & (y > 0)).any() else 0.0

        # 오른쪽이 더 침범 → 왼쪽(+)으로
        corr = self.wa_gain * (right - left)
        corr = float(np.clip(corr, -self.wa_max, self.wa_max))

        if abs(corr) > 0.02:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._wa_warned > 1.0:
                self._wa_warned = now
                side = '우측' if corr > 0 else '좌측'
                self.get_logger().info(
                    f'[wall_avoid] {side} 벽 접근 — '
                    f'침범 R{right:.02f}/L{left:.02f}m → '
                    f'보정 {math.degrees(corr):+.1f}°')
        return self._wa_smooth(corr)

    def _wa_smooth(self, corr: float) -> float:
        """회피 조향 저역통과 — 스캔 노이즈로 조향이 튀지 않게."""
        a = float(np.clip(self.wa_lpf, 0.0, 1.0))
        self._wa_prev = a * corr + (1.0 - a) * self._wa_prev
        return self._wa_prev

    def _scan_cb(self, msg: LaserScan):
        # 벽 회피는 pose 없이도 (차량 좌표계만으로) 동작해야 한다 —
        # AMCL 수렴 전에도 벽에 스치는 건 막아야 하므로 pose 검사보다 먼저 저장.
        self.latest_scan = msg

        if self.pos is None:
            return
        if self.mode != 'q2':
            return          # 아래 격자 판정은 Q2 전용

        p = self.get_parameter

        # ── 빔별 기대거리 비교 (기본 경로) ─────────────────────────────
        # 격자+마스크 방식은 "벽 근처인가" 만 보므로 AMCL 오차가 track_margin 을
        # 넘으면 벽이 장애물이 된다 (2026-08-05 q2: 직선 9.4m 를 42cm 밀려 주행).
        # 여기서는 "맵 벽보다 앞에서 끊긴 빔" 만 장애물로 본다.
        if getattr(self, 'ray_occ', None) is not None:
            pts_world = self._raycast_obstacle_pts(msg)
            if len(pts_world) == 0:
                self.on_track_pts = np.zeros((0, 2))
                self.blocking_pts = []
                self.free_votes.append(True)
                return
            # 마스크를 다시 걸지 않는다 — 이미 "맵으로 설명되지 않는 점" 만 남았다.
            self._classify_obstacles(pts_world, 0.0, masked=False)
            return

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

        # ★ 격자는 base_link 기준인데 ranges 는 laser 원점에서 잰 값이다.
        #   라이다가 base_link 보다 0.27m 앞에 있으므로(bringup_launch.py 의
        #   static_baselink_to_laser) 그 차이만큼 빼고 비교해야 한다.
        #   빼지 않으면 |rho - range| 에 0.27m 의 계통 오차가 생긴다. safe 가
        #   0.3 이라 정면에선 간신히 걸쳐 셀 1~2 개만 살아남고, 옆으로 갈수록
        #   오차가 커져 아예 안 잡힌다 — 실차에서 "점 1개"로 나타난 원인.
        lx = gx - self.laser_dx
        ly = gy - self.laser_dy
        rho = np.sqrt(lx**2 + ly**2)
        phi = np.arctan2(ly, lx)
        idx = np.clip(
            ((phi - msg.angle_min) / msg.angle_increment).astype(int),
            0, len(ranges) - 1)
        occ = (np.abs(rho - ranges[idx]) < safe).ravel()

        gxf = gx.ravel()[occ]
        gyf = gy.ravel()[occ]

        if len(gxf) == 0:
            self.blocking_pts = []
            self.free_votes.append(True)
            return

        # rotate + translate to world frame
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        R  = np.array([[cy, -sy], [sy, cy]])
        pts_world = (R @ np.vstack((gxf, gyf))).T + self.pos

        self._classify_obstacles(pts_world, ign)

    def _classify_obstacles(self, pts_world: np.ndarray, ign: float,
                            masked: bool = True):
        # without bound files → treat all grid hits as candidates (벽도 섞인다)
        if not masked or getattr(self, 'track_mask', None) is None:
            on_track = list(pts_world)
        else:
            # 사전계산된 마스크 조회 — 예전의 점당 cv2.pointPolygonTest 2회
            # 파이썬 루프를 대체한다 (_build_track_mask 주석 참조).
            h, w = self.map_meta[0], self.map_meta[1]
            res, ox, oy = self.map_meta[2], self.map_meta[3], self.map_meta[4]
            pts_img = _world_to_img(pts_world, h, res, ox, oy)
            ix, iy = pts_img[:, 0], pts_img[:, 1]
            inb = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            keep = np.zeros(len(pts_world), bool)
            keep[inb] = self.track_mask[iy[inb], ix[inb]]
            on_track = list(pts_world[keep])

        # Frenet 후보 평가는 "지금 진로" 밖의 점도 봐야 한다 (옆으로 피한 자리에
        # 뭐가 있는지). blocking 만으로는 그 정보가 없어서 따로 보관한다.
        self.on_track_pts = (np.asarray(on_track) if on_track
                             else np.zeros((0, 2)))

        if not on_track:
            self.blocking_pts = []
            self.free_votes.append(True)
            return

        #   그리고 "막고 있는 점"만 남겨 회피 방향·마커에 쓴다 — 예전엔 트랙 안의
        #   모든 점 평균으로 방향을 정해서 엉뚱한 쪽으로 틀 수 있었다.
        # ★ 판정 기준은 "내가 실제로 지나갈 자리를 막았는가" 하나다.
        #   레이싱라인은 우리가 임의로 넣은 계획일 뿐이고, 충돌은 실제 진로에서만
        #   일어난다. 라인 기준으로 보면 추종 오차만큼 벌어진 구간에서 실제로
        #   부딪힐 장애물을 통째로 놓친다 (2026-07-21 실차: 두 번째 장애물).
        #   라인은 "어디로 피할지"(_obstacle_shift 의 목표점)에만 쓴다.
        blocking = [p for p, k in
                    zip(on_track, self._in_predicted_corridor(np.asarray(on_track)))
                    if k]

        self.blocking_pts = blocking
        self.free_votes.append(not blocking)
        if blocking:
            self._pub_obstacle_markers(blocking)

    # ── publishers / helpers ───────────────────────────────────────────────

    def _publish_cmd(self, steer: float, speed: float):
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
        msg.drive.speed          = speed
        self.drive_pub.publish(msg)

    def _publish_racing_line(self):
        """전체 웨이포인트를 속도 컬러 LINE_STRIP 으로 발행 (RViz 레이싱라인)."""
        if self.wps is None:
            return
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = 'racing_line'; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.05                      # 선 두께 (m)
        m.pose.orientation.w = 1.0
        vmin = float(self.wps[:, 2].min())
        vmax = float(self.wps[:, 2].max())
        span = max(vmax - vmin, 1e-6)
        for x, y, v in self.wps:
            m.points.append(Point(x=float(x), y=float(y), z=0.0))
            t = (float(v) - vmin) / span      # 0=느림(빨강) ~ 1=빠름(초록)
            m.colors.append(ColorRGBA(r=float(1.0 - t), g=float(t), b=0.0, a=1.0))
        # 순환 경로 — 시작점을 이어 닫는다
        m.points.append(Point(x=float(self.wps[0, 0]), y=float(self.wps[0, 1]), z=0.0))
        m.colors.append(m.colors[0])
        self.path_pub.publish(m)

    def _update_car_trail(self, x: float, y: float):
        """차가 지나온 궤적을 Path 로 발행 (0.1m 이상 이동 시에만 점 추가)."""
        if self._last_trail is not None and \
                math.hypot(x - self._last_trail[0], y - self._last_trail[1]) < 0.1:
            return
        self._last_trail = (x, y)
        now = self.get_clock().now()

        # 파일 쓰기는 여기서 하지 않는다 — 50Hz 제어 루프 안이라 디스크 I/O를
        # 매 포인트마다 넣으면 지연 위험 (예전 _scan_cb 44.5ms 지연 버그와
        # 같은 종류). 메모리에만 쌓고, 실제 저장은 노드 종료 시 save_run_log()에서
        # 한 번에 flush 한다.
        t = (now - self._run_t0).nanoseconds / 1e9
        self._trail_log.append((t, x, y, float(self.speed)))
        if len(self._trail_log) > 3000:
            self._trail_log.pop(0)

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = now.to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        self._car_path.poses.append(ps)
        if len(self._car_path.poses) > 3000:   # 메모리 상한 (한 바퀴 400점 ×여러 랩)
            self._car_path.poses.pop(0)
        self._car_path.header.stamp = ps.header.stamp
        self.carpath_pub.publish(self._car_path)

    def save_run_log(self):
        """주행 종료 시 1회만 호출 — 실주행 경로 + 그 시점 게인(race_script.yaml)을
        logs/ 밑에 팀 공유 폴더로 저장한다 (개인용 chat_log/ 와 다름, gitignore 안 됨).

        웨이포인트 CSV 자체는 복사하지 않는다 — gains.yaml 안에 이미
        map_name/waypoints 필드로 "어떤 웨이포인트를 썼는지" 경로가 남으므로,
        나중에 driven_path.csv 와 그 경로를 함께 읽어 비교하면 된다.
        """
        if not self._trail_log:
            return
        ts = time.strftime('%Y%m%d_%H%M%S')
        # ★ 폴더명에 컨트롤러까지 넣는다 — q2 를 stanley/pure_pursuit 둘 다로
        #   돌리게 된 뒤로는 mode 만으론 어느 주행인지 구분이 안 된다.
        #   gains.yaml 을 열어봐야 알던 정보를 폴더명만 보고 바로 알 수 있게.
        run_dir = os.path.join(_REPO_ROOT, 'logs', f'{ts}_{self.mode}_stanley')
        try:
            os.makedirs(run_dir, exist_ok=True)

            with open(os.path.join(run_dir, 'driven_path.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'x', 'y', 'speed'])
                for t, x, y, v in self._trail_log:
                    w.writerow([f'{t:.3f}', f'{x:.4f}', f'{y:.4f}', f'{v:.3f}'])

            race_script = os.environ.get('RACE_SCRIPT',
                                          os.path.expanduser('~/race_script.yaml'))
            if os.path.exists(race_script):
                with open(race_script) as f:
                    cfg_text = f.read()
                with open(os.path.join(run_dir, 'gains.yaml'), 'w') as f:
                    f.write(cfg_text)

            self.get_logger().info(
                f'[run_log] {len(self._trail_log)}점, {run_dir} 에 저장')
        except Exception as e:
            self.get_logger().warn(f'[run_log] 저장 실패: {e}')

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
        # Ctrl-C 로 컨텍스트가 이미 닫힌 뒤면 발행이 예외를 던진다.
        # 종료 경로에서만 나는 노이즈라 조건을 두고 조용히 넘긴다.
        if rclpy.ok():
            node._publish_cmd(0.0, 0.0)
        node.save_run_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
