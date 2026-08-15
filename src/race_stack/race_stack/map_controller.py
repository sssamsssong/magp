#!/usr/bin/env python3
"""
MAP (Model-based Adaptive Pure pursuit) 컨트롤러 — 실차판.

  ※ 이름의 'MAP' 은 트랙 맵(map_name)과 무관하다. 논문/구현 이름 그대로다.
    참고: ETH-PBL MAP-Controller (github.com/ETH-PBL/MAP-Controller)

시뮬 번들의 f1tenth_racing/map_controller_node.py 를 실차 스택으로 옮긴 것.
**조향·속도 알고리즘은 시뮬판과 동일하게 유지한다** — 시뮬에서 튠한 값을 그대로
가져와 실차와 비교하는 게 이 이식의 목적이기 때문이다. 바뀐 것은 배관뿐:

  시뮬판                          →  실차판(이 파일)
  ─────────────────────────────────────────────────────────────────
  /ego_racecar/odom 에서 pose     →  AMCL 보정 ⊕ odom 추측항법 융합 pose
  상수 하드코딩                    →  race_script.yaml [map] 절 파라미터
  odom 콜백에서 /drive 발행        →  50Hz 타이머 고정 발행 (아래 ★ 참조)
  input() 로 시작 대기             →  /race/enabled (mission_manager 래치)
  속도 클램프 없음                 →  mode 별 max_speed/min_speed 로 클램프

★ pose 는 왜 /amcl_pose 를 그냥 쓰지 않는가 (2026-08-06 실차: 출발선에서 정지)
  AMCL 이 /amcl_pose 를 내는 시점은 (1) 첫 스캔 1회, (2) 그 뒤로는 2.5cm/1.7도
  이상 움직였을 때뿐이다. race.launch.py 가 컨트롤러를 8초 지연 기동하므로 (1)은
  이미 지나가 있고, 차가 멈춰 있으니 (2)도 영원히 안 온다 →
  "pose 없음 → 0속도 발행 → 안 움직임 → pose 없음" 데드락.
  (첫 시험에서 정확히 이 증상이 나왔다 — 손으로 밀어주면 그때부터 정상 주행)

  그래서 stanley_controller 와 같은 방식을 쓴다:
    · 첫 odom 이 오면 init_pose 에 있다고 **가정하고 시드** — 기다림에 주행을 걸지
      않는다. AMCL 도 같은 값으로 파티클을 뿌리므로 가정이 일치한다.
    · 그 뒤 pose = (map←odom 보정) ⊕ (최신 odom) 으로 50Hz 추측항법
    · /amcl_pose 가 오면 보정 변환만 갱신 — 제어 주기와 분리된다
  이 융합은 사인파 진동에도 직결된다. AMCL 원시 pose 는 이산적으로 튀는데,
  그 위에서 짧은 L1 으로 조향하면 튐이 그대로 조향에 실린다.

알고리즘 요약:
  1. 쿠빅 스플라인으로 경로 곡률 κ 사전 계산
  2. CSV 속도 → 역방향 스윕(제동가속도 a_brake)으로 감속 프로파일 생성
  3. L1 가이던스로 필요한 횡가속도 a_lat 계산
  4. Pacejka 타이어 모델 역산으로 앞바퀴 슬립각 α_f 를 구해 기구학 조향각에 더함
  5. 조향 1차 저역통과 필터

구독: /amcl_pose, <odom_topic>, /race/enabled
발행: /drive, /waypoints_path, /waypoint, /car_path
"""
import csv
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, ColorRGBA, Float64

from race_stack.speed_profile import (
    solve_profile, LaunchLimiter, preview_speed, steer_speed_limit)
from visualization_msgs.msg import Marker

from race_stack.stanley_controller import _REPO_ROOT, _normalize, write_run_config

# /race/enabled 는 mission_manager 가 래치(TRANSIENT_LOCAL)로 발행한다.
# 기본 QoS 로 구독하면 컨트롤러가 8초 뒤에 뜨는 사이 나간 값을 놓쳐서,
# 출발 게이트가 걸어둔 enabled=false 를 못 받고 그냥 달려나간다.
LATCHED_IN = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

GRAVITY = 9.81


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapController(Node):
    def __init__(self):
        super().__init__('map_controller')

        # ── 배관 파라미터 ────────────────────────────────────────────────
        self.declare_parameter('mode', 'q1')          # 로그 폴더명 용도
        self.declare_parameter('odom_topic', '/odom_imu')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('min_speed', 0.5)

        # ── 차량 기하 (pure_pursuit 절과 공유) ───────────────────────────
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steer', 0.36)

        # ── L1 가이던스 ─────────────────────────────────────────────────
        # ★ 시뮬 기본값(0.2725 / 0.2750)을 실차에 그대로 쓰면 안 된다.
        #   조향각은 delta_kin = atan(2·wheelbase·sin(eta) / L1) 이라 L1 이
        #   짧을수록 이득이 폭증한다. L1=0.41m(1.5m/s) 이면
        #     atan(2·0.33·sin(eta)/0.41) = atan(1.61·sin eta)
        #   → 겨우 13도의 방위 오차에서 max_steer(0.36rad)가 포화된다.
        #   시뮬은 pose 가 완벽해서 오차가 안 쌓이지만, 실차는 AMCL 튐만으로도
        #   그 13도를 넘나들어 좌우 풀조향이 반복된다 = 사인파 진동
        #   (2026-08-06 실차 저속 시험에서 확인). 그래서 실차 기본값을 키운다.
        self.declare_parameter('l1_min', 0.90)
        self.declare_parameter('l1_k',   0.35)

        # ── 차량 동역학 ─────────────────────────────────────────────────
        # ★ mass/lf/lr 은 조향 결과에 영향을 주지 않는다 — F_yf/(mu·Fzf) 를 계산할
        #   때 대수적으로 약분되어 a_lat/(mu·g) 만 남는다. 하중이동을 넣게 될 때를
        #   대비해 형태만 남겨둔 것이고, 실제로 조향을 바꾸는 건 mu 하나다.
        # 실측 (2026-08-09): m=3.354kg, 앞1510g/뒤1840g, L=0.32m
        self.declare_parameter('mass', 3.354)
        self.declare_parameter('lf',   0.1757)
        self.declare_parameter('lr',   0.1441)
        # ★ mu 는 마찰계수가 아니다 — **실효 최대 그립 = mu × pac_D** 다.
        #   MAP 논문 III-B: "peak value D̃ is in fact μ·Fz,i·Di".
        #   실측 실효 그립 0.40g(화강석+먼지) → mu = 0.40 / 1.15 = 0.348.
        #   ※ gen_raceline 의 --mu 는 a_lat_max = mu·g 라 규약이 달라 0.40 이다.
        #      두 값이 달라 보이는 건 정상이다. 헷갈리면 pac_D 를 같이 볼 것.
        #   대회 우레탄은 다르니 현장에서 다시 잴 것. race_script.yaml map.mu 가 원본.
        self.declare_parameter('mu',   0.348)

        # ── Frenet 회피 (frenet_planner 노드가 /frenet/offset 로 준다) ──────
        #   MAP 은 회피 로직을 직접 갖지 않는다 — Stanley 가 쓰던 것과 **같은 코드**를
        #   독립 노드로 돌리고 여기서는 결과(횡오프셋 d)만 받는다.
        #
        #   ★ 타임아웃 폴백이 핵심이다. 노드가 죽거나 늦으면 d=0(레이싱라인)으로
        #     돌아가야 한다. 조용히 옛 오프셋을 유지하면 장애물이 치워진 뒤에도
        #     라인을 벗어난 채 달린다.
        #
        #   ★ 램프(shift_rate)도 필수다. 기준선을 계단으로 옮기면 목표점이 튀고
        #     그게 곧 조향 킥이 된다 (Stanley 에서 "휘청휘청" 으로 관측된 것).
        self.declare_parameter('frenet_enabled',    True)
        self.declare_parameter('frenet_timeout',    0.5)   # s
        self.declare_parameter('frenet_shift_rate', 0.9)   # m/s

        # ── Pacejka Magic Formula ───────────────────────────────────────
        # 우리 차 추정 (2026-08-09): F1TENTH 폼 타이어 기반, MAP 논문 대비
        #   B 소폭 상향(차량 경량으로 접지압 낮음), D 소폭 하향(타이어 상태 불확실)
        self.declare_parameter('pac_b',  3.8)
        self.declare_parameter('pac_c',  1.3)
        self.declare_parameter('pac_d',  1.15)
        self.declare_parameter('pac_e', -0.5)

        # ── 속도 프로파일 ───────────────────────────────────────────────
        # ── 속도 프로파일 ────────────────────────────────────────────────
        # ★ grip_util(η) 이 유일한 튜닝 노브다. 속도를 배수로 스케일하지 않는다.
        #   속도를 k 배 하면 요구 횡가속은 **k² 배** 가 되어 비선형이고, 어느
        #   코너가 한계인지까지 바뀐다 — 코너에 맞추면 직선이 느리고 직선에
        #   맞추면 코너가 터진다. 어떤 k 로도 동시에 못 맞춘다.
        #   대신 **마찰 예산**에 η 를 곱한다: a_lat_max = η·μ_eff·g.
        #   v ∝ √η 라 코너는 √η 배로 줄지만, 직선은 a_lon 과 구간 길이가
        #   지배하므로 균일하게 안 줄어든다 — 그게 물리적으로 맞다.
        self.declare_parameter('grip_util',   0.6)
        # 물리와 무관한 안전 천장. 0 = 끔(기본). μ 나 η 가 틀려도 이 속도는
        # 안 넘는다는 마지막 방어선이 필요할 때만 켠다.
        self.declare_parameter('hard_cap',    0.0)

        # 종방향 상한 [m/s²]. 0 이하면 물리에서 자동 유도한다:
        #   가속 — 후륜구동이라 **후축 하중분율**이 한계다.  a = μ_eff·g·lf/L
        #   제동 — 네 바퀴를 다 쓰므로 타이어 예산 전체.      a = μ_eff·g
        #   둘 다 마찰원 안으로 잘린다(min).
        self.declare_parameter('a_lon_acc',   0.0)
        self.declare_parameter('a_lon_brk',   0.0)

        # ── 출발 가속 제한 ────────────────────────────────────────────
        #   정지에서 출발하는 첫 몇 초만 가속을 따로 더 조인다. 프로파일의
        #   a_lon_acc 와 별개로 두는 이유는 **출발이 특별하기 때문**이다.
        #
        #   2026-08-10 실측 (bag 16개, odom 이동거리 ÷ AMCL 이동거리):
        #     같은 가속도인데도 출발 5초 안쪽이 더 미끄러진다
        #        가속도      출발 0~5s   5s 이후    차이
        #        0.0~0.8      1.0332     1.0246    +0.86%
        #        0.8~1.6      1.0344     1.0268    +0.76%
        #        1.6 이상     1.0694     1.0360    +3.34%   ← 두 배
        #     정지마찰 깨기·구동계 백래시·하중 뒤쏠림이 겹치는 구간이다.
        #     그 결과 출발 5초에 **허위거리 87cm** 가 쌓인다 (odom 17.35m,
        #     AMCL 16.35m). 종방향은 복도에서 관측이 거의 안 되므로(앞뒤 0.2~1.0cm
        #     vs 좌우 6~7cm) AMCL 이 다 못 되돌리고 자세에 남는다 →
        #     "실제보다 더 갔다"고 믿어 **첫 코너를 일찍 돈다.**
        #
        #   launch_acc 에서 시작해 launch_time 에 걸쳐 a_lon_acc 로 선형 복귀한다
        #   (계단이 생기지 않게). 감속에는 절대 걸지 않는다. 0 = 끔.
        #   ※ 공짜가 아니다 — 1랩 +1.16s (0.8㎧²/3s, fun_clean η=0.6).
        #     누적되지는 않는다(3랩 +1.20s). 자세한 스윕은 race_script.yaml.
        self.declare_parameter('launch_acc',  0.0)
        self.declare_parameter('launch_time', 3.0)

        # 구동계 지연 [s]. 지령→실측 속도의 상호상관으로 실측한 값(2026-08-09:
        # 250~290ms, 고속에선 410ms). _profile_speed 가 이 지연 **과** 제동거리
        # 중 긴 쪽을 본다. 0 = 지연항만 끔 (제동거리 미리보기는 a_brk 로 남는다).
        self.declare_parameter('speed_lag',   0.25)

        # ★ 박스 위 벤치 모드. true 면 **발행 직전에 속도를 0 으로 못박는다**
        #   (_publish_cmd). 조향은 살아 있어야 검증이 되므로 건드리지 않는다.
        self.declare_parameter('bench',       False)

        # 슬립보정(α_f) 이득 [0~1]. 1.0 = 모델값 그대로.
        #   ★ 2026-08-09 실차: 코너에서 α_f 가 기구학항만큼 커져 **과조향**이 났다.
        #     주행 궤적 곡률이 기준의 1.2~1.9배 (s=20~28m 구간, cool_clean).
        #     타이어가 모델만큼 안 미끄러진다는 뜻 — mu 나 pac_b 가 실제보다
        #     낮게 잡혀 있다. 둘 다 [미검증] 값이다.
        #     mu 를 올리면 속도 프로파일까지 같이 빨라져 원인이 섞이므로,
        #     **조향 보정만** 따로 줄일 수 있는 손잡이를 둔다.
        self.declare_parameter('slip_gain',   1.0)

        # ── AMCL 보정 슬루 제한 ──────────────────────────────────────
        #   ★ 2026-08-09 실차: q2 로 장애물을 치고 다니며 **트랙 구조물이 실제로
        #     움직였다.** 맵-스캔 잔차가 평균 4.1 → 8.2cm, 최대 15 → 39cm 로 두 배.
        #     그 상태로 q1 을 6㎧ 로 돌리자 AMCL 이 s=17.6m 에서 2.90m,
        #     s=18.6m 에서 **4.50m** 점프했다 (yaw 는 52.9°). 그 자리가 정확히
        #     주행이 이탈한 지점이다.
        #
        #   예전 pose_cb 는 새 보정을 **그대로 즉시** 반영했다. 4.5m 점프가 다음
        #   50Hz 틱에서 그대로 조향 오차가 된다. 게다가 오늘 steer_alpha 를
        #   0.8892(170ms) → 0.555(34ms) 로 낮추면서, 그 점프를 **우연히 뭉개주던
        #   필터마저 얇아졌다.** 요 진동은 잡혔지만 pose 점프에는 더 취약해졌다.
        #
        #   그래서 보정을 순간에 반영하지 않고 **속도 제한**을 건다. 위치추정이
        #   틀린 채로 잠깐 가는 것보다, 조향이 튀어 벽으로 가는 게 훨씬 나쁘다.
        #   거부(reject)가 아니라 제한이라 AMCL 이 옳으면 몇 초 안에 수렴한다.
        #   0 = 끔(옛 동작).
        #   한계값은 bag 에서 분포를 재서 정했다 — AMCL 은 평상시에도 파티클
        #   잡음으로 보정을 흔든다(정상 주행 p99 3.63㎧). 너무 조이면 위치추정을
        #   망가뜨리고, 너무 풀면 점프가 그대로 통과한다. 4.0㎧/120°/s 에서
        #   정상 주행의 0.8% 만 걸리고 4.5m 점프는 1.1s 에 걸쳐 반영된다.
        self.declare_parameter('pose_corr_rate',     4.0)   # m/s
        self.declare_parameter('pose_corr_yaw_rate', 2.1)   # rad/s

        # ↓ 폐기됨. 남겨둔 건 옛 설정이 죽지 않게 하려는 것뿐이고, 1.0 이 아니면
        #   경고하고 **무시한다**. 위 grip_util 을 쓸 것.
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('a_brake',     0.0)   # 폐기 — a_lon_brk 로 대체
        # 마찰원 곡률 캡 v ≤ sqrt(mu·g/κ). 시뮬판은 이 캡이 꺼져 있어서
        # 기본값도 false 로 맞춘다 (시뮬↔실차 비교의 기준을 같게 하려고).
        # 고속 웨이포인트를 그대로 쓰면서 코너만 물리 한계로 눌러보고 싶을 때 켠다.
        # ↓ 폐기 — 곡률 한계는 이제 **항상** 걸린다 (마찰원 프로파일의 1단계).
        #   예전엔 기본이 False 라 코너 속도가 순전히 CSV(μ≈1.05 가정)에서 왔다.
        self.declare_parameter('mu_cap', True)

        # ── 조향 필터 ───────────────────────────────────────────────────
        # 1차 LPF 계수 (0=끔, 1에 가까울수록 무겁다). 시뮬판 기본 0.1981.
        self.declare_parameter('steer_alpha', 0.1981)

        # 웨이포인트 진행 방향이 차 헤딩과 반대면 배열을 뒤집는다 (첫 pose 때 1회)
        self.declare_parameter('reverse_check', True)

        # AMCL 첫 pose 를 받기 전까지의 위치 가정 (AMCL 과 같은 값을 쓴다)
        self.declare_parameter('init_x',   0.0)
        self.declare_parameter('init_y',   0.0)
        self.declare_parameter('init_yaw', 0.0)

        self.mode = self.get_parameter('mode').value

        # ── 웨이포인트 로드 + 사전 계산 ─────────────────────────────────
        self._bench = bool(self.get_parameter('bench').value)
        if self._bench:
            self.get_logger().warn(
                '★ BENCH 모드 — 속도를 0 으로 강제한다 (바퀴가 돌지 않는다). '
                '조향은 정상 동작하므로 박스 위에서 확인할 수 있다.')

        wp_file = self.get_parameter('waypoint_file').value
        self._wp_ds = 0.1
        self._a_acc = 0.0
        self._a_brk = 0.0
        self.wp = self._load_waypoints(wp_file)
        if self.wp is None:
            self.get_logger().error(
                f'웨이포인트 파일을 못 읽음: "{wp_file}" — map_controller 정지 상태. '
                f'./race.sh log <이름> 으로 웨이포인트를 먼저 기록하세요.')
        else:
            self.get_logger().info(f'웨이포인트 {len(self.wp)}개 로드')
            _d = np.hypot(np.diff(self.wp[:, 0]), np.diff(self.wp[:, 1]))
            self._wp_ds = float(np.median(_d)) if len(_d) else 0.1
            self._curvature = self._compute_curvature()
            self._v_profile = self._compute_speed_profile()
        self._pac_alpha_lut, self._pac_Fy_lut = self._pacejka_lut()

        # ── 주행 상태 ───────────────────────────────────────────────────
        self.enabled = False    # ★ mission_manager 의 /race/enabled 를 받아야 켜진다
        self._launch = LaunchLimiter()   # 출발 직후 가속 제한 (휠 스핀 방지)
        self.pos = None
        self.yaw = 0.0
        self.speed = 0.0        # odom 실측 속도 — L1 거리·조향 계산에 쓴다
        self._near_idx = 0
        self._initialized = False
        self._steer_prev = 0.0

        # ── pose 융합 상태 (stanley_controller 와 같은 방식) ─────────────
        self._amcl_pose = None       # 마지막 /amcl_pose 원시값
        self._amcl_seen = False
        self._odom_last = None       # (t, x, y, yaw)
        self._odom_buf = deque(maxlen=100)
        self._map_from_odom = None   # (th, tx, ty) — map←odom 보정 변환
        self._corr_t = time.time()   # 마지막 보정 갱신 시각 (슬루 제한용)
        self._corr_clamped = 0       # 제한이 걸린 횟수 — 주행 후 요약에 쓴다
        self._seed_pose = (float(self.p('init_x')), float(self.p('init_y')),
                           float(self.p('init_yaw')))

        # ── 실주행 경로 로그 ────────────────────────────────────────────
        self._trail_log = []
        self._last_trail = None
        self._run_t0 = self.get_clock().now()
        self._car_path = Path()
        self._car_path.header.frame_id = 'map'

        # ── ROS 인터페이스 ──────────────────────────────────────────────
        self.drive_pub   = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.path_pub    = self.create_publisher(Marker, '/waypoints_path', 10)
        self.wp_pub      = self.create_publisher(Marker, '/waypoint',       10)
        self.carpath_pub = self.create_publisher(Path,   '/car_path',       10)

        self.create_subscription(Bool, '/race/enabled', self.enabled_cb, LATCHED_IN)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.pose_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)

        # Frenet 회피 오프셋
        self._fr_d_cmd  = 0.0     # 노드가 보낸 목표 오프셋
        self._fr_rx_t   = None    # 마지막 수신 시각 (타임아웃 판정)
        self._fr_shift  = 0.0     # 실제로 적용 중인 값 (램프됨)
        self._fr_warned = False
        self._fr_spd    = 1.0     # 속도 스케일 (1대1). 타임아웃이면 1.0 으로 복귀
        self._fr_spd_t  = None
        if bool(self.p('frenet_enabled')):
            self.create_subscription(Float64, '/frenet/offset',
                                     self._frenet_cb, 10)
            # ★ 1대1 — 통로가 좁으면 줄이고 넓으면 안 줄인다. 프로파일 **위로는
            #   절대 안 올린다** (그건 마찰원을 넘는 것이다). 발행이 끊기면
            #   1.0 으로 돌아간다 — 오프셋과 같은 타임아웃 폴백 규약.
            self.create_subscription(Float64, '/frenet/speed_scale',
                                     self._frenet_spd_cb, 10)

        # 제어·발행을 50Hz 로 고정한다. AMCL 콜백에 걸어두면 주기가 들쭉날쭉하고
        # (2.5cm 이동마다) 정지 상태에서는 아예 돌지 않는다.
        self.create_timer(0.02, self._cmd_tick)
        self.create_timer(1.0, self._publish_racing_line)

        self.get_logger().info(
            f'MapController ready (mode={self.mode} '
            f'grip_util={self.p("grip_util"):.2f})')

    def p(self, name):
        return self.get_parameter(name).value

    # ── 웨이포인트 ─────────────────────────────────────────────────────
    def _load_waypoints(self, path):
        """x,y[,speed] CSV. 헤더('x,y,speed')와 '#' 주석 줄은 건너뛴다."""
        if not path or not os.path.exists(path):
            return None
        pts = []
        try:
            with open(path) as f:
                for row in csv.reader(f):
                    if not row or row[0].strip().startswith('#'):
                        continue
                    try:
                        x, y = float(row[0]), float(row[1])
                    except ValueError:
                        continue          # 헤더 줄
                    v = float(row[2]) if len(row) > 2 else -1.0
                    pts.append((x, y, v))
        except OSError:
            return None
        return np.array(pts) if len(pts) >= 3 else None

    # ── 사전 계산: 쿠빅 스플라인 곡률 ──────────────────────────────────
    def _compute_curvature(self):
        pts = self.wp[:, :2]
        try:
            from scipy.interpolate import CubicSpline
            ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            s = np.concatenate([[0.0], np.cumsum(ds)])
            # 시작점을 뒤에 한 번 더 붙여 주기 스플라인으로 닫는다.
            s_ext = np.append(s, s[-1] + float(np.linalg.norm(pts[0] - pts[-1])))
            if np.any(np.diff(s_ext) <= 1e-9):
                raise ValueError('중복 웨이포인트 — 스플라인 불가')
            cs_x = CubicSpline(s_ext, np.append(pts[:, 0], pts[0, 0]),
                               bc_type='periodic')
            cs_y = CubicSpline(s_ext, np.append(pts[:, 1], pts[0, 1]),
                               bc_type='periodic')
            dx, dy = cs_x(s, 1), cs_y(s, 1)
            ddx, ddy = cs_x(s, 2), cs_y(s, 2)
        except Exception as e:
            # 웨이포인트가 촘촘하면 유한차분도 충분하다. 스플라인이 실패했다고
            # 컨트롤러 전체가 죽는 편보다 낫다.
            self.get_logger().warn(f'스플라인 곡률 실패({e}) → 유한차분으로 대체')
            fwd = np.roll(pts, -1, axis=0) - pts
            bwd = pts - np.roll(pts, 1, axis=0)
            dx, dy = (fwd[:, 0] + bwd[:, 0]) / 2.0, (fwd[:, 1] + bwd[:, 1]) / 2.0
            ddx = fwd[:, 0] - bwd[:, 0]
            ddy = fwd[:, 1] - bwd[:, 1]

        denom = np.maximum((dx ** 2 + dy ** 2) ** 1.5, 1e-6)
        k = np.abs(dx * ddy - dy * ddx) / denom
        self.get_logger().info(
            f'곡률 사전 계산: max={k.max():.3f} mean={k.mean():.3f} 1/m')
        return k

    # ── 사전 계산: 역방향 스윕 속도 프로파일 ───────────────────────────
    def _compute_speed_profile(self):
        """마찰원 결합 forward-backward. 로직은 speed_profile 모듈 (컨트롤러 공용)."""
        v, info = solve_profile(
            self.wp[:, 0], self.wp[:, 1], self.wp[:, 2],
            mu_eff=float(self.p('mu')) * float(self.p('pac_d')),
            grip_util=float(self.p('grip_util')),
            lf=float(self.p('lf')), lr=float(self.p('lr')),
            v_min=float(self.p('min_speed')),
            a_lon_acc=float(self.p('a_lon_acc')),
            a_lon_brk=float(self.p('a_lon_brk')),
            hard_cap=float(self.p('hard_cap')),
            curvature=self._curvature)

        if abs(float(self.p('speed_scale')) - 1.0) > 1e-6:
            self.get_logger().error(
                f"speed_scale={self.p('speed_scale')} 은 **무시된다** (폐기됨). "
                f"속도를 배수로 스케일하면 요구 횡가속이 제곱으로 변해 어느 코너가 "
                f"한계인지까지 바뀐다. grip_util 로 마찰 예산을 조절할 것.")
        if float(self.p('a_brake')) > 0.0:
            self.get_logger().warn("a_brake 는 폐기됐다 — a_lon_brk 를 쓸 것.")

        self._a_acc = float(info['a_acc'])   # 출발 제한이 여기로 복귀한다
        self._a_brk = float(info['a_brk'])   # 제동 미리보기
        self.get_logger().info(
            f"속도 프로파일 η={info['eta']:.2f}  예산 {info['a_g']:.2f}㎧² "
            f"(가속 {info['a_acc']:.2f} / 제동 {info['a_brk']:.2f})")
        self.get_logger().info(
            f"  v {info['v_min']:.2f}~{info['v_max']:.2f}㎧ (중앙 {info['v_med']:.2f}) "
            f"| 예상 랩 {info['lap_time']:.1f}s"
            + (f" | ⚠ hard_cap 에 걸린 구간 {100*info['capped']:.0f}%"
               if info['capped'] > 0.01 else ''))

        # ★ 조향 쪽 유효값도 같이 찍는다 (2026-08-09).
        #   yaml → launch → declare_parameter 3단 폴백이라, 코드에 적힌 기본값을
        #   보고 "지금 이 값이겠거니" 하고 오해하기 쉽다. 실제로 map_controller 의
        #   declare_parameter(slip_gain, 1.0) 을 보고 1.0 이 걸린 줄 안 적이 있다.
        #   **적용된 값은 여기서 확인할 것.** 오늘만 이 종류 함정이 다섯 번 나왔다.
        a = float(self.p('steer_alpha'))
        tau = -0.02 / math.log(a) if 0.0 < a < 1.0 else 0.0
        self.get_logger().info(
            # ★ max() 다, 덧셈이 아니다 (_steer_map 과 같은 식). 2026-08-13 까지
            #   여기가 '+' 로 찍혀 있어서 로그만 보고 L1 을 2배로 오해했다.
            #   바로 위 주석이 "적용된 값은 여기서 확인할 것" 이라고 해놓고
            #   정작 이 줄이 거짓말을 하고 있었다.
            f"  조향 l1=max({self.p('l1_min'):.3f}, {self.p('l1_k'):.3f}·v)  "
            f"alpha={a:.3f}(τ{tau*1000:.0f}ms)  slip_gain={self.p('slip_gain'):.2f}  "
            f"speed_lag={self.p('speed_lag'):.2f}s  max_steer={self.p('max_steer'):.3f}")
        return v

    def _profile_speed(self, near, v):
        """프로파일에서 지령 속도를 뽑되 **제동거리와 구동 지연 중 긴 쪽**을 본다.

        지연만 보면 v·t_lag 앞만 감속한다. 제동거리 v²/(2 a_brk) 가 더 길면
        그걸 쓴다. a_brk 는 프로파일(오버레이 a_lon_brk, 0 이면 자동) 그대로.
        가속은 앞당기지 않는다.
        """
        return preview_speed(
            self._v_profile, near, v, self._wp_ds,
            lag=float(self.p('speed_lag')), a_brk=self._a_brk)

    def _launch_limit(self, speed, dt):
        """출발 가속 제한. 로직은 speed_profile.LaunchLimiter (컨트롤러 공용).
        여기서는 파라미터를 넘기고 로그만 남긴다."""
        if not self._launch.active:
            return speed
        now = self.get_clock().now().nanoseconds * 1e-9
        lim = self._launch.apply(speed, now, dt,
                                 float(self.p('launch_acc')),
                                 float(self.p('launch_time')),
                                 self._a_acc)
        if lim < speed - 1e-3:
            self.get_logger().info(
                f'[출발제한] 상한 {self._launch.cap:.2f}㎧²  '
                f'{speed:.2f}→{lim:.2f}㎧', throttle_duration_sec=0.5)
        return lim

    def _friction_cap(self):
        """마찰원 곡률 한계속도. (진단용 — 프로파일 계산은 _compute_speed_profile 안에서)"""
        return np.sqrt(float(self.p('mu')) * GRAVITY
                       / np.maximum(self._curvature, 1e-4))

    # ── 사전 계산: Pacejka LUT (단조 구간 ±0.3 rad) ────────────────────
    def _pacejka_lut(self):
        B, C, D, E = (float(self.p('pac_b')), float(self.p('pac_c')),
                      float(self.p('pac_d')), float(self.p('pac_e')))
        alphas = np.linspace(-0.30, 0.30, 500)
        Fy = D * np.sin(C * np.arctan(
            B * alphas - E * (B * alphas - np.arctan(B * alphas))))
        self.get_logger().info(f'Pacejka LUT: Fy=[{Fy[0]:.3f}, {Fy[-1]:.3f}]')
        return alphas, Fy

    def _pacejka_inv(self, Fy_norm_req: float) -> float:
        clamped = float(np.clip(Fy_norm_req,
                                self._pac_Fy_lut[0] * 0.99,
                                self._pac_Fy_lut[-1] * 0.99))
        return float(np.interp(clamped, self._pac_Fy_lut, self._pac_alpha_lut))

    # ── MAP 조향각 ─────────────────────────────────────────────────────
    def _frenet_spd_cb(self, msg: Float64):
        self._fr_spd = float(np.clip(msg.data, 0.2, 1.0))
        self._fr_spd_t = self.get_clock().now().nanoseconds * 1e-9

    def _frenet_speed_scale(self) -> float:
        if not bool(self.p('frenet_enabled')) or self._fr_spd_t is None:
            return 1.0
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._fr_spd_t > float(self.p('frenet_timeout')):
            return 1.0
        return self._fr_spd

    def _frenet_cb(self, msg: Float64):
        self._fr_d_cmd = float(msg.data)
        self._fr_rx_t = self.get_clock().now().nanoseconds * 1e-9
        if self._fr_warned:
            self._fr_warned = False
            self.get_logger().info('frenet 오프셋 수신 재개')

    def _frenet_shift(self, goal, near):
        """목표점을 경로 법선 방향으로 램프된 오프셋만큼 민다.

        법선은 **경로** 법선을 쓴다 — 차량 헤딩 법선을 쓰면 회피 조향으로 차가
        돌 때마다 시프트 방향까지 같이 돌아 되먹임이 생긴다 (Stanley 주석과 같은 이유).
        """
        if not bool(self.p('frenet_enabled')):
            return goal
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = (self._fr_rx_t is not None
                 and now - self._fr_rx_t <= float(self.p('frenet_timeout')))
        target = self._fr_d_cmd if fresh else 0.0
        if not fresh and not self._fr_warned and self._fr_rx_t is not None:
            self._fr_warned = True
            self.get_logger().warn(
                '/frenet/offset 끊김 — 레이싱라인으로 복귀한다 (회피 없음)',
                throttle_duration_sec=2.0)

        # ★★ **한 번도** 못 받은 경우를 따로 경고한다 (2026-08-15).
        #   예전엔 `_fr_rx_t is not None` 조건 때문에 이 경우가 조용히 지나갔다.
        #   frenet_planner 가 기동에서 죽으면 오프셋이 아예 안 오는데,
        #   컨트롤러는 d=0 으로 정상 주행하므로 **로그상 완전히 정상으로 보인다.**
        #   실제로 그랬다 — 08/15 race 주행에서 planner 가 NameError 로 즉사했고
        #   차는 레이싱라인만 따라 달렸다. "회피를 전혀 못 한다" 로만 보였고
        #   원인을 짚을 단서가 로그 어디에도 없었다.
        #   회피를 켜 놓고 오프셋이 안 오면 그건 항상 고장이다.
        if self._fr_rx_t is None:
            self.get_logger().error(
                '/frenet/offset 을 한 번도 못 받았다 — frenet_planner 가 죽었거나 '
                '안 떴다. 회피가 전혀 동작하지 않는 상태다. '
                'ros2 node list 로 frenet_planner 확인할 것',
                throttle_duration_sec=3.0)

        step = float(self.p('frenet_shift_rate')) / 50.0     # _cmd_tick 50Hz
        self._fr_shift += float(np.clip(target - self._fr_shift, -step, step))
        if abs(self._fr_shift) < 1e-3:
            return goal

        n = len(self.wp)
        fwd = self.wp[(near + 1) % n, :2]
        bwd = self.wp[(near - 1) % n, :2]
        tang = fwd - bwd
        tn = float(np.hypot(tang[0], tang[1]))
        if tn < 1e-9:
            return goal
        left = np.array([-tang[1], tang[0]]) / tn
        return goal + self._fr_shift * left

    def _steer_map(self, pos, yaw, goal) -> float:
        v = max(self.speed, 0.3)
        L1 = max(float(self.p('l1_min')), float(self.p('l1_k')) * v)

        dx, dy = goal[0] - pos[0], goal[1] - pos[1]
        bearing = math.atan2(dy, dx) - yaw
        eta = math.atan2(math.sin(bearing), math.cos(bearing))

        # L1 가이던스가 요구하는 횡가속도
        a_lat = 2.0 * v * v * math.sin(eta) / L1

        wb = float(self.p('wheelbase'))
        mass, lr, mu = float(self.p('mass')), float(self.p('lr')), float(self.p('mu'))
        Fzf = mass * GRAVITY * lr / wb          # 앞축 수직하중
        F_yf = mass * a_lat * lr / wb           # 앞축 필요 횡력
        alpha_f = self._pacejka_inv(F_yf / (mu * Fzf))   # 슬립각 보정
        alpha_f *= float(self.p('slip_gain'))            # ★ 과조향 억제 (위 주석)

        delta_kin = math.atan(wb * 2.0 * math.sin(eta) / max(L1, 1e-3))
        max_steer = float(self.p('max_steer'))
        return float(np.clip(delta_kin + alpha_f, -max_steer, max_steer))

    # ── Lookahead 포인트 탐색 ──────────────────────────────────────────
    def _find_lookahead(self, pos, lookahead, yaw):
        n = len(self.wp)
        dists = np.linalg.norm(self.wp[:, :2] - pos, axis=1)

        if not self._initialized:
            near = int(np.argmin(dists))
            if self.p('reverse_check'):
                fwd = np.array([math.cos(yaw), math.sin(yaw)])
                wp_dir = self.wp[(near + 1) % n, :2] - self.wp[near, :2]
                if float(np.dot(fwd, wp_dir)) < 0:
                    self.get_logger().warn('웨이포인트 방향 역전 감지 → 배열 역전')
                    self.wp = self.wp[::-1].copy()
                    self._curvature = self._curvature[::-1].copy()
                    self._v_profile = self._v_profile[::-1].copy()
                    dists = dists[::-1].copy()
                    near = n - 1 - near
            self._initialized = True
        else:
            # 지역 창 탐색 — 경로가 자기 근처를 다시 지나는 구간에서 전역 argmin 이
            # 반대편 점으로 튀는 걸 막는다. 다만 AMCL 이 크게 점프하면 창 안에
            # 가까운 점이 없으므로 그때만 전역 재획득한다.
            win = [(self._near_idx + i) % n for i in range(-10, 50)]
            near = win[int(np.argmin(dists[win]))]
            if dists[near] > 1.0:
                near = int(np.argmin(dists))
                self.get_logger().warn(
                    f'경로에서 {dists[near]:.2f}m 이탈 → 최근접 인덱스 재획득',
                    throttle_duration_sec=2.0)

        self._near_idx = near

        # 호길이를 따라 lookahead 만큼 전진한 지점(선분 보간)
        arc = 0.0
        for i in range(n):
            i0, i1 = (near + i) % n, (near + i + 1) % n
            seg = float(np.linalg.norm(self.wp[i1, :2] - self.wp[i0, :2]))
            if arc + seg >= lookahead:
                a = float(np.clip((lookahead - arc) / max(seg, 1e-6), 0.0, 1.0))
                return (1.0 - a) * self.wp[i0, :2] + a * self.wp[i1, :2], near
            arc += seg
        return None, near

    # ── 콜백 ───────────────────────────────────────────────────────────
    def enabled_cb(self, msg: Bool):
        if self.enabled and not msg.data:
            self._steer_prev = 0.0      # 재활성화 시 이전 조향 재사용 방지
            self.publish_stop()
        if msg.data and not self.enabled:
            # 정지 → 출발. 여기서부터 launch_time 동안 가속을 조인다.
            self._launch.start(self.get_clock().now().nanoseconds * 1e-9,
                               self.speed)
        self.enabled = msg.data

    def odom_cb(self, msg: Odometry):
        self.speed = abs(float(msg.twist.twist.linear.x))

        p = msg.pose.pose
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        entry = (t, p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self._odom_last = entry
        self._odom_buf.append(entry)

        # ── 초기 pose 시드 ────────────────────────────────────────────
        # AMCL 첫 pose 를 기다리지 않는다 (파일 상단 ★ 주석의 데드락).
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

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """AMCL 수신 → map←odom 보정 변환만 갱신. 제어는 _cmd_tick 이 50Hz 로."""
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self._amcl_pose = (px, py, yaw)
        if not self._amcl_seen:
            self._amcl_seen = True
            sx, sy, _ = self._seed_pose
            self.get_logger().info(
                f'AMCL pose 첫 수신 ({px:.3f}, {py:.3f}) — 시드 가정 해제. '
                f'시드와의 차이 {math.hypot(px - sx, py - sy):.2f}m')

        # AMCL 측정 시각의 odom 을 **보간**해서 보정을 계산한다.
        #
        # ★ 예전엔 가장 가까운 샘플을 그냥 썼는데, /odom_imu 가 50Hz 라 평균
        #   4.1ms 가 어긋난다. 5㎧ 면 그 사이 차가 2.06cm 를 가고, 그게 그대로
        #   보정 오차가 된다 — **속도에 정비례**한다.
        #   2026-08-10 bag 분석: 보정 점프가 v 에 선형(R²=0.972)이었고,
        #   5㎧ 에서 관측 2.83cm 중 2.06cm(73%)가 이 양자화로 설명됐다.
        #   기울기도 실측 3.6mm/㎧ vs 예측 4.1mm/㎧ 로 맞았다.
        #   보간하면 이 항이 사라진다 (남는 건 파티클 잡음 ~0.9cm).
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        od = self._odom_at(t)
        if od is None:
            return
        ox, oy, oyaw = od
        th = _normalize(yaw - oyaw)
        c, s = math.cos(th), math.sin(th)
        target = (th, px - (c * ox - s * oy), py - (s * ox + c * oy))

        self._map_from_odom = self._slew_correction(target)

    def _odom_at(self, t):
        """시각 t 의 odom 자세를 선형 보간해서 돌려준다. (x, y, yaw)

        버퍼는 (t, x, y, yaw) 오름차순이다. t 가 버퍼 밖이면 가장 가까운 끝을
        그대로 쓴다 — 외삽하지 않는다 (AMCL 이 한참 늦게 오면 외삽이 더 위험하다).
        """
        buf = self._odom_buf
        if not buf:
            if self._odom_last is None:
                return None
            _, ox, oy, oyaw = self._odom_last
            return (ox, oy, oyaw)
        if t <= buf[0][0]:
            _, ox, oy, oyaw = buf[0]
            return (ox, oy, oyaw)
        if t >= buf[-1][0]:
            _, ox, oy, oyaw = buf[-1]
            return (ox, oy, oyaw)
        lo, hi = 0, len(buf) - 1
        while hi - lo > 1:                      # 이분 탐색
            mid = (lo + hi) // 2
            if buf[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0, w0 = buf[lo]
        t1, x1, y1, w1 = buf[hi]
        r = (t - t0) / max(t1 - t0, 1e-9)
        dw = _normalize(w1 - w0)                # yaw 는 wrap 처리 후 보간
        return (x0 + (x1 - x0) * r,
                y0 + (y1 - y0) * r,
                _normalize(w0 + dw * r))

    def _slew_correction(self, target):
        """보정 변환을 목표로 **제한된 속도로** 옮긴다. 자세한 이유는
        pose_corr_rate 파라미터 주석 참고.

        변환 계수(th,tx,ty)를 직접 자르지 않는다 — th 가 조금만 변해도 원점에서
        먼 곳의 위치는 크게 움직이기 때문이다. **지금 위치가 얼마나 뛰는지**를
        기준으로 자른 뒤 그 위치를 만드는 변환을 되계산한다.
        """
        rate = float(self.p('pose_corr_rate'))
        yrate = float(self.p('pose_corr_yaw_rate'))
        now = time.time()
        prev = self._map_from_odom
        dt = min(max(now - self._corr_t, 1e-3), 0.5)
        self._corr_t = now

        # 출발 전에는 자유롭게 수렴시킨다 — 초기 정렬을 막으면 안 된다
        if prev is None or rate <= 0.0 or not self.enabled or self._odom_last is None:
            return target

        def apply(T, o):
            th, tx, ty = T
            _, ox, oy, oyaw = o
            c, s = math.cos(th), math.sin(th)
            return (tx + c * ox - s * oy, ty + s * ox + c * oy, _normalize(th + oyaw))

        o = self._odom_last
        x0, y0, w0 = apply(prev,   o)
        x1, y1, w1 = apply(target, o)
        d = math.hypot(x1 - x0, y1 - y0)
        dw = _normalize(w1 - w0)
        lim, wlim = rate * dt, yrate * dt
        if d <= lim and abs(dw) <= wlim:
            return target

        k = min(1.0, lim / d) if d > 1e-9 else 1.0
        kw = min(1.0, wlim / abs(dw)) if abs(dw) > 1e-9 else 1.0
        xt, yt = x0 + (x1 - x0) * k, y0 + (y1 - y0) * k
        wt = _normalize(w0 + dw * kw)

        self._corr_clamped += 1
        self.get_logger().warn(
            f'AMCL 보정 {d*100:.0f}cm / {math.degrees(abs(dw)):.0f}° 를 '
            f'{lim*100:.0f}cm / {math.degrees(wlim):.0f}° 로 제한 '
            f'(누적 {self._corr_clamped}회) — 맵과 실제가 어긋났을 수 있다',
            throttle_duration_sec=2.0)

        _, ox2, oy2, oyaw2 = o
        th2 = _normalize(wt - oyaw2)
        c2, s2 = math.cos(th2), math.sin(th2)
        return (th2, xt - (c2 * ox2 - s2 * oy2), yt - (s2 * ox2 + c2 * oy2))

    def _current_pose(self):
        """AMCL 보정 ⊕ 최신 odom = 고주기 pose. 보정이 없으면 AMCL 원시값 폴백."""
        if self._map_from_odom is not None and self._odom_last is not None:
            th, tx, ty = self._map_from_odom
            _, ox, oy, oyaw = self._odom_last
            c, s = math.cos(th), math.sin(th)
            return (tx + c * ox - s * oy,
                    ty + s * ox + c * oy,
                    _normalize(th + oyaw))
        return self._amcl_pose

    def _cmd_tick(self):
        """50Hz 제어 + 발행. 멈춰 있을 때는 그 이유를 로그로 남긴다 —
        "로그는 정상인데 차가 안 감" 을 다시 겪지 않도록."""
        pose = self._current_pose()
        if not self.enabled or self.wp is None or pose is None:
            self._publish_cmd(0.0, 0.0)
            why = ('/race/enabled=false (mission_manager 가 껐음)' if not self.enabled
                   else '웨이포인트 없음' if self.wp is None
                   else 'pose 없음 — odom 미수신 (imu_odom 이 떠 있는지 확인)')
            self.get_logger().warn(f'정지 중: {why}', throttle_duration_sec=3.0)
            return

        if not self._amcl_seen:
            self.get_logger().warn(
                'AMCL pose 미수신 — init_pose 가정으로 주행 중입니다. '
                '실제 위치와 다르면 라인을 벗어납니다',
                throttle_duration_sec=5.0)

        px, py, yaw = pose
        self.pos = np.array([px, py])
        self.yaw = yaw

        v = max(self.speed, 0.3)
        L1 = max(float(self.p('l1_min')), float(self.p('l1_k')) * v)

        goal, near = self._find_lookahead(self.pos, L1, yaw)
        if goal is None:
            self._publish_cmd(0.0, 0.0)
            return

        goal = self._frenet_shift(goal, near)
        steer = self._steer_map(self.pos, yaw, goal)
        a = float(self.p('steer_alpha'))
        steer = a * self._steer_prev + (1.0 - a) * steer
        self._steer_prev = steer

        # 프로파일이 이미 물리(마찰원)와 min_speed·hard_cap 을 다 반영했다.
        # 여기서 다시 max_speed 로 자르면 상한이 부활해 등속 주행이 된다.
        speed = self._profile_speed(near, v)
        # 1대1 회피 감속 (막히면 줄이고 넓으면 그대로). 출발제한보다 **먼저**
        # 걸어야 한다 — 뒤에 걸면 출발제한의 적분 상태(_cmd_prev)가 스케일된
        # 값을 못 보고 다음 틱에서 다시 튄다.
        speed *= self._frenet_speed_scale()
        speed = self._launch_limit(speed, 0.02)   # 타이머 주기 50Hz
        # 조향이 포화되면 그 곡률을 마찰 예산 안에서 돌 수 있는 속도로 자른다.
        v_lock = steer_speed_limit(
            steer, float(self.p('wheelbase')),
            float(self.p('mu')) * float(self.p('pac_d')),
            float(self.p('grip_util')))
        if v_lock < speed:
            speed = v_lock

        self.get_logger().info(
            f'[MAP] κ={self._curvature[near]:.3f}  '
            f'steer={math.degrees(steer):.1f}°  spd={speed:.2f}m/s  '
            f'(실측 {self.speed:.2f})',
            throttle_duration_sec=0.5)

        self._publish_cmd(steer, speed)
        self._pub_wp_marker(goal)
        # ★ 실측(self.speed)과 지령(speed)을 **둘 다** 남긴다 (2026-08-09).
        #   예전엔 지령만 'speed' 열로 저장해서, 사후 분석에서 그걸 실측으로
        #   착각하면 '속도 추종 지연' 같은 없는 문제를 만들어낸다. 실제로 그랬다.
        self._update_car_trail(px, py, self.speed, steer, speed, near)

    def _publish_cmd(self, steer: float, speed: float):
        """정지 중에도 계속 흘린다 — /drive 가 0.2s 끊기면 ackermann_mux 가
        명령을 버리고, vesc_to_odom 도 odom kick 을 못 받는다.

        종료 중(컨텍스트 닫힘)에는 조용히 빠진다. 안 그러면 마지막 타이머 콜백이
        RCLError 를 던져 종료가 트레이스백으로 지저분해진다."""
        if not rclpy.ok():
            return
        # ★★ bench(박스 위) — 여기가 유일한 강제 지점이다. 절대 옮기거나 지우지 말 것.
        #   2026-08-09: 예전엔 bench 가 max_speed=0 을 주고 컨트롤러의
        #   clip(v, min_speed, max_speed) 에 기대고 있었다. 속도 개편에서 그 클램프를
        #   없애자 `./race.sh box` 가 **풀속도로 주행**했다 (start_gate 도 함께 꺼져
        #   있어 시작 버튼조차 없었다). 안전 기능을 다른 코드가 우연히 하는 일에
        #   얹으면 안 된다 — 그래서 발행 직전에 못박는다.
        if self._bench:
            speed = 0.0
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.steering_angle = float(steer)
        out.drive.speed = float(speed)
        self.drive_pub.publish(out)

    def publish_stop(self):
        self._publish_cmd(0.0, 0.0)

    # ── 시각화 (stanley 와 같은 토픽/형식 — rviz.sh 그대로 쓴다) ────────
    def _publish_racing_line(self):
        if self.wp is None or not rclpy.ok():
            return
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'racing_line'; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.05
        m.pose.orientation.w = 1.0
        # ★ CSV 속도가 아니라 실제로 명령할 v_profile 로 색을 칠한다 —
        #   역방향 스윕과 모드 상한이 어디를 눌렀는지 RViz 에서 바로 보인다.
        vmin, vmax = float(self._v_profile.min()), float(self._v_profile.max())
        span = max(vmax - vmin, 1e-6)
        for (x, y, _), v in zip(self.wp, self._v_profile):
            m.points.append(Point(x=float(x), y=float(y), z=0.0))
            t = (float(v) - vmin) / span      # 0=느림(빨강) ~ 1=빠름(초록)
            m.colors.append(ColorRGBA(r=float(1.0 - t), g=float(t), b=0.0, a=1.0))
        m.points.append(Point(x=float(self.wp[0, 0]), y=float(self.wp[0, 1]), z=0.0))
        m.colors.append(m.colors[0])
        self.path_pub.publish(m)

    def _pub_wp_marker(self, target):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'target_wp'; m.id = 0
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position.x = float(target[0])
        m.pose.position.y = float(target[1])
        m.pose.orientation.w = 1.0
        m.color.r = m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.lifetime.nanosec = int(1e8)
        self.wp_pub.publish(m)

    def _update_car_trail(self, x: float, y: float, speed_meas: float,
                          steer: float = 0.0, cmd_v: float = 0.0,
                          near: int = -1):
        """0.1m 이상 움직였을 때만 점 추가. 디스크 쓰기는 종료 시 한 번에.

        ★ 2026-08-09: 진단 채널을 늘렸다. 위치·실측속도만으로는
          "직선→코너 전환부 좌우 흔들림" 과 "지령을 못 따라가는 속도" 를
          가릴 수 없었다 — 조향 신호가 없으면 제어 루프 진동인지 그립
          포화인지 구분이 안 되고, 지령속도가 없으면 추종 지연을 못 잰다.
        """
        if self._last_trail is not None and \
                math.hypot(x - self._last_trail[0], y - self._last_trail[1]) < 0.1:
            return
        self._last_trail = (x, y)
        now = self.get_clock().now()
        t = (now - self._run_t0).nanoseconds / 1e9
        self._trail_log.append((t, x, y, float(speed_meas), float(steer),
                                float(cmd_v), float(self.yaw), int(near)))
        if len(self._trail_log) > 3000:
            self._trail_log.pop(0)

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = now.to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        self._car_path.poses.append(ps)
        if len(self._car_path.poses) > 3000:
            self._car_path.poses.pop(0)
        self._car_path.header.stamp = ps.header.stamp
        self.carpath_pub.publish(self._car_path)

    def save_run_log(self):
        """주행 종료 시 1회 — 실주행 경로 + 그 시점 race_script.yaml 스냅샷을
        logs/<시각>_<모드>_map/ 에 저장 (stanley/pure_pursuit 와 같은 포맷)."""
        if not self._trail_log:
            return
        ts = time.strftime('%Y%m%d_%H%M%S')
        run_dir = os.path.join(_REPO_ROOT, 'logs', f'{ts}_{self.mode}_map')
        try:
            os.makedirs(run_dir, exist_ok=True)
            with open(os.path.join(run_dir, 'driven_path.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                # steer/cmd_v/yaw 는 2026-08-09 추가 — 진동(제어 루프)과
                # 속도 추종 지연을 사후에 가리려면 지령 신호가 있어야 한다.
                # speed = **실측**(odom), cmd_v = **지령**(프로파일). 헷갈리지 말 것.
                w.writerow(['t', 'x', 'y', 'speed', 'steer', 'cmd_v', 'yaw', 'near'])
                for r in self._trail_log:
                    w.writerow([f'{r[0]:.3f}', f'{r[1]:.4f}', f'{r[2]:.4f}',
                                f'{r[3]:.3f}', f'{r[4]:.4f}', f'{r[5]:.3f}',
                                f'{r[6]:.4f}', r[7]])

            # ★ race_script.yaml + 차량 오버레이를 함께 남긴다.
            #   오버레이가 빠지면 스냅샷이 조용히 거짓말을 한다 (2026-08-11).
            write_run_config(run_dir)

            self.get_logger().info(
                f'[run_log] {len(self._trail_log)}점, {run_dir} 에 저장')
        except Exception as e:
            self.get_logger().warn(f'[run_log] 저장 실패: {e}')


def main():
    rclpy.init()
    node = MapController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM(race.sh kill_stack)은 rclpy 가 컨텍스트를 내리면서
        # ExternalShutdownException 으로 올라온다. KeyboardInterrupt 만 잡으면
        # 이게 안 잡혀 종료 정리가 반쪽이 된다.
        pass
    except Exception:
        # 컨텍스트가 이미 내려간 뒤 executor 내부(wait set 등)에서 나는 예외는
        # 종료 잡음일 뿐이다. 살아 있는 상태의 예외만 그대로 올린다.
        # (RCLError 는 rclpy 의 private 모듈에만 있어 타입으로 못 잡는다)
        if rclpy.ok():
            raise
    finally:
        # ★ 컨텍스트가 이미 닫힌 뒤면 발행이 예외를 던진다. 가드 없이 두면 그
        #   예외가 finally 를 뚫고 나가 **save_run_log 가 실행되지 않는다** —
        #   2026-08-06 에 MAP·pure_pursuit 주행 로그가 하나도 안 남던 진짜 원인.
        #   (stanley 는 이 가드가 있어서 그 주행만 로그가 남아 있었다)
        if rclpy.ok():
            node.publish_stop()
        node.save_run_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
