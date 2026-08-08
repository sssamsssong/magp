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
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker

from race_stack.stanley_controller import _REPO_ROOT, _normalize

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
        # mu: 대회(우레탄)=1.0~1.3, 연습(화강석)=0.8 권장
        self.declare_parameter('mu',   1.0)

        # ── Pacejka Magic Formula ───────────────────────────────────────
        # 우리 차 추정 (2026-08-09): F1TENTH 폼 타이어 기반, MAP 논문 대비
        #   B 소폭 상향(차량 경량으로 접지압 낮음), D 소폭 하향(타이어 상태 불확실)
        self.declare_parameter('pac_b',  3.8)
        self.declare_parameter('pac_c',  1.3)
        self.declare_parameter('pac_d',  1.15)
        self.declare_parameter('pac_e', -0.5)

        # ── 속도 프로파일 ───────────────────────────────────────────────
        self.declare_parameter('a_brake',     6.0902)
        self.declare_parameter('speed_scale', 1.0)
        # 마찰원 곡률 캡 v ≤ sqrt(mu·g/κ). 시뮬판은 이 캡이 꺼져 있어서
        # 기본값도 false 로 맞춘다 (시뮬↔실차 비교의 기준을 같게 하려고).
        # 고속 웨이포인트를 그대로 쓰면서 코너만 물리 한계로 눌러보고 싶을 때 켠다.
        self.declare_parameter('mu_cap', False)

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
        wp_file = self.get_parameter('waypoint_file').value
        self.wp = self._load_waypoints(wp_file)
        if self.wp is None:
            self.get_logger().error(
                f'웨이포인트 파일을 못 읽음: "{wp_file}" — map_controller 정지 상태. '
                f'./race.sh log <이름> 으로 웨이포인트를 먼저 기록하세요.')
        else:
            self.get_logger().info(f'웨이포인트 {len(self.wp)}개 로드')
            self._curvature = self._compute_curvature()
            self._v_profile = self._compute_speed_profile()
        self._pac_alpha_lut, self._pac_Fy_lut = self._pacejka_lut()

        # ── 주행 상태 ───────────────────────────────────────────────────
        self.enabled = False    # ★ mission_manager 의 /race/enabled 를 받아야 켜진다
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

        # 제어·발행을 50Hz 로 고정한다. AMCL 콜백에 걸어두면 주기가 들쭉날쭉하고
        # (2.5cm 이동마다) 정지 상태에서는 아예 돌지 않는다.
        self.create_timer(0.02, self._cmd_tick)
        self.create_timer(1.0, self._publish_racing_line)

        self.get_logger().info(
            f'MapController ready (mode={self.mode} '
            f'max_speed={self.p("max_speed"):.2f} mu_cap='
            f'{"on" if self.p("mu_cap") else "off"})')

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
        n = len(self.wp)
        ds = np.linalg.norm(np.roll(self.wp[:, :2], -1, axis=0) - self.wp[:, :2],
                            axis=1)

        v_max = float(self.p('max_speed'))
        v_min = float(self.p('min_speed'))
        csv_v = self.wp[:, 2] * float(self.p('speed_scale'))

        if np.any(self.wp[:, 2] <= 0.0):
            # speed 열이 없는 CSV — 곡률만으로 프로파일을 만든다.
            self.get_logger().warn(
                'CSV 에 speed 열이 없음 → 마찰원 곡률 캡으로 속도 프로파일 생성')
            v = self._friction_cap()
        else:
            v = csv_v.copy()
            if self.p('mu_cap'):
                v = np.minimum(v, self._friction_cap())

        # ★ 모드 상한을 스윕 **전에** 건다. 나중에 자르면 8m/s 로 만든 감속선이
        #   실제 주행속도와 안 맞아 코너 진입 브레이킹이 헛돈다.
        v = np.clip(v, v_min, v_max)

        # 역방향 스윕 — 다음 점 속도에서 a_brake 로 거슬러 올라간 값 이하로 제한
        a_brake = float(self.p('a_brake'))
        for _ in range(3):
            for i in range(n - 1, -1, -1):
                j = (i + 1) % n
                v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2.0 * a_brake * ds[i]))
        v = np.clip(v, v_min, v_max)

        self.get_logger().info(
            f'속도 프로파일: {v.min():.2f}~{v.max():.2f} m/s '
            f'(CSV {csv_v.min():.2f}~{csv_v.max():.2f}, 상한 {v_max:.2f})')
        return v

    def _friction_cap(self):
        """마찰원 한계속도 v = sqrt(mu·g/κ)."""
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

        # AMCL 측정 시각과 가장 가까운 odom 샘플로 보정을 계산한다
        # (AMCL 값은 그 시각의 pose 이므로 그 시각의 odom 과 짝지어야 정확)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        od = (min(self._odom_buf, key=lambda e: abs(e[0] - t))
              if self._odom_buf else self._odom_last)
        if od is None:
            return
        _, ox, oy, oyaw = od
        th = _normalize(yaw - oyaw)
        c, s = math.cos(th), math.sin(th)
        self._map_from_odom = (th, px - (c * ox - s * oy),
                               py - (s * ox + c * oy))

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

        steer = self._steer_map(self.pos, yaw, goal)
        a = float(self.p('steer_alpha'))
        steer = a * self._steer_prev + (1.0 - a) * steer
        self._steer_prev = steer

        speed = float(np.clip(self._v_profile[near],
                              self.p('min_speed'), self.p('max_speed')))

        self.get_logger().info(
            f'[MAP] κ={self._curvature[near]:.3f}  '
            f'steer={math.degrees(steer):.1f}°  spd={speed:.2f}m/s  '
            f'(실측 {self.speed:.2f})',
            throttle_duration_sec=0.5)

        self._publish_cmd(steer, speed)
        self._pub_wp_marker(goal)
        self._update_car_trail(px, py, speed)

    def _publish_cmd(self, steer: float, speed: float):
        """정지 중에도 계속 흘린다 — /drive 가 0.2s 끊기면 ackermann_mux 가
        명령을 버리고, vesc_to_odom 도 odom kick 을 못 받는다.

        종료 중(컨텍스트 닫힘)에는 조용히 빠진다. 안 그러면 마지막 타이머 콜백이
        RCLError 를 던져 종료가 트레이스백으로 지저분해진다."""
        if not rclpy.ok():
            return
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

    def _update_car_trail(self, x: float, y: float, speed: float):
        """0.1m 이상 움직였을 때만 점 추가. 디스크 쓰기는 종료 시 한 번에."""
        if self._last_trail is not None and \
                math.hypot(x - self._last_trail[0], y - self._last_trail[1]) < 0.1:
            return
        self._last_trail = (x, y)
        now = self.get_clock().now()
        t = (now - self._run_t0).nanoseconds / 1e9
        self._trail_log.append((t, x, y, float(speed)))
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
