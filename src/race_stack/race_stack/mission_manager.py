#!/usr/bin/env python3
"""
Mission Manager — 랩 카운트 + 제한시간 + 자동 정지.

/amcl_pose로 출발선(start_x, start_y, start_radius) 통과를 감지해
랩을 센다. 설정 랩수 완주 또는 제한시간 도달 시 /race/enabled=False를
발행해 컨트롤러를 멈춘다 (Fully Autonomous: 사람이 손 안 대고 자동 정지).

발행: /race/enabled (Bool, latched), /race/lap (Int32)
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, Int32
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('laps', 0)            # 0 = 무제한
        self.declare_parameter('time_limit_sec', 0)  # 0 = 무제한


        # ── 예선 시퀀스 (2026-08-21, car1 d285065 이식) ────────────────────
        # Q1→buffer→Q2→buffer→Q3 를 **한 프로세스 안에서** 이어간다. 사람이
        # 단계마다 다시 눌러줘야 하면 그 자체로 Fully Autonomous(규정 5.5)가
        # 깨지므로 자동 진행이 필수다. 단계 전환마다 map_controller 의
        # grip_util 과 frenet_planner 의 avoid_enabled 를 /set_parameters 로 바꾼다.
        self.declare_parameter('sequence_enabled',    False)
        self.declare_parameter('seq_grip_util_q1',     0.3)
        self.declare_parameter('seq_grip_util_buffer', 0.15)
        self.declare_parameter('seq_grip_util_q2',     0.5)
        self.declare_parameter('seq_grip_util_q3',     0.7)
        # ── Q3 속도 램프 ────────────────────────────────────────────────
        #   Q3 는 기록 주행이라 제일 빠르게 돌고 싶은데, 첫 바퀴부터 목표
        #   속도로 들어가면 아직 타이어·노면이 안 잡힌 상태라 위험하다.
        #   q3_start 에서 시작해 **한 바퀴마다 seq_grip_step_q3 만큼** 올려
        #   목표(seq_grip_util_q3)에 닿으면 그 값으로 계속 돈다.
        #     예) 목표 0.8, 시작 0.6, 스텝 0.1
        #         1바퀴 0.6 → 2바퀴 0.7 → 3바퀴 0.8 → 이후 0.8 유지
        #   0 이거나 목표 이상이면 램프를 안 쓰고 처음부터 목표값으로 간다.
        self.declare_parameter('seq_grip_util_q3_start', 0.0)
        self.declare_parameter('seq_grip_step_q3',       0.1)
        self.declare_parameter('seq_laps_q1',          3)
        self.declare_parameter('seq_laps_buffer',      1)
        self.declare_parameter('seq_laps_q2',          3)
        # ★ quali-mini 용 (2026-08-24). seq_laps_buffer 를 0 으로 주면 안 된다 —
        #   아래 self.seq 구성에서 각 단계는 laps>0 이어야 랩 완주로 다음
        #   단계로 넘어간다(pose_cb 참고). 0 은 "무제한" 취급 관례(mission.laps
        #   등)와 겹쳐서, BUFFER 단계가 laps=0 이면 **영원히 안 끝난다**
        #   (BUFFER 의 time_limit 도 0 이라 시간으로도 못 넘어간다). 그래서
        #   버퍼를 없애려면 랩수를 0으로 주지 말고, 이 스위치로 BUFFER1/2
        #   단계 자체를 시퀀스에서 뺀다.
        self.declare_parameter('seq_skip_buffer',      False)

        self.declare_parameter('seq_time_q3',           75)
        # ── 대회 규정 (2026-08-22 확인) ──────────────────────────────────
        #   · Q1 출발부터 Q3 종료까지 **총 4분(240s)** 을 넘으면 안 된다
        #   · Q3 **단독으로도 1분(60s)** 을 넘으면 안 된다
        #   ※ 여기 기본값은 규정 + 15초(255/75)다. race_script.yaml 의
        #     mission 절 주석에 그렇게 정한 이유가 적혀 있다.
        #   두 제약을 동시에 만족시키려면 Q3 시간을 고정으로 둘 수 없다:
        #       Q3 실제 시간 = min(seq_time_q3, seq_total_limit − 지금까지 경과)
        #   Q1~BUFFER2 가 3분 안에 끝났으면 Q3 는 온전히 1분을 쓰고(총 ≤4분),
        #   3분을 넘겼으면 남은 시간만 써서 총합을 정확히 4분에 맞춘다.
        self.declare_parameter('seq_total_limit',      255)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_radius', 1.5)
        # ── 출발 게이트 ────────────────────────────────────────────────
        # 대회에서는 출발선에 세워둔 채 스택을 미리 띄우고, 출발 신호에 맞춰
        # 바로 나가야 한다. 기동에 8초가 걸리므로 신호를 듣고 실행하면 늦는다.
        # 게이트를 켜면 /race/enabled=false 로 잡아두고 트리거를 기다린다.
        #   · 터미널 Enter (같은 터미널. /dev/tty 를 직접 열어 읽는다)
        #   · 조이스틱 버튼 (F710 기준 B=1)
        #   · 다른 터미널에서 ./race.sh go
        # 부수 효과로 **AMCL 이 수렴할 시간**이 생긴다 — 출발과 동시에 수렴을
        # 시작하던 예전 방식보다 안정적이다.
        self.declare_parameter('start_gate',   False)
        self.declare_parameter('start_button', 1)      # F710: A=0 B=1 X=2 Y=3
        self.declare_parameter('start_key',    True)   # 터미널 Enter 허용

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enabled_pub = self.create_publisher(Bool, '/race/enabled', latched)
        self.lap_pub = self.create_publisher(Int32, '/race/lap', latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.pose_cb, 10)

        self.lap = 0
        self.inside = True          # 시작 시 출발선 안에 있다고 가정
        self.armed = False          # 한 번 출발선을 벗어나야 카운트 시작
        self.start_time = self.get_clock().now()
        self.finished = False
        self.lap_start = self.start_time
        # ★ 단계 제한시간 기준. lap_start 와 **반드시 별개**여야 한다 —
        #   lap_start 는 랩이 끝날 때마다 갱신되므로(개별 랩타임 표시용),
        #   Q3 처럼 랩이 아니라 시간이 기준인 단계에서 그걸 쓰면 랩이 끝날
        #   때마다 제한시간이 다시 시작된다.
        self.stage_start = self.start_time
        self._stage_limit = 0.0     # 이 단계에 실제로 주어진 시간 [s] (0=랩 기준)
        self._grip_now = 0.0        # 지금 map_controller 에 걸어둔 grip_util

        # ── 예선 시퀀스 상태 ───────────────────────────────────────────
        self.seq_enabled = bool(self.p('sequence_enabled'))
        self.seq = []
        self.stage_idx = 0
        self.map_cli = self.fr_cli = None
        if self.seq_enabled:
            self.seq = [
                {'name': 'Q1',      'laps': int(self.p('seq_laps_q1')),
                 'time_limit': 0, 'grip_util': float(self.p('seq_grip_util_q1')),
                 'avoid': False},
                {'name': 'BUFFER1', 'laps': int(self.p('seq_laps_buffer')),
                 'time_limit': 0, 'grip_util': float(self.p('seq_grip_util_buffer')),
                 'avoid': False},
                {'name': 'Q2',      'laps': int(self.p('seq_laps_q2')),
                 'time_limit': 0, 'grip_util': float(self.p('seq_grip_util_q2')),
                 'avoid': True},
                {'name': 'BUFFER2', 'laps': int(self.p('seq_laps_buffer')),
                 'time_limit': 0, 'grip_util': float(self.p('seq_grip_util_buffer')),
                 'avoid': False},
                {'name': 'Q3',      'laps': 0,
                 'time_limit': int(self.p('seq_time_q3')),
                 'grip_util': float(self.p('seq_grip_util_q3')),
                 'grip_start': float(self.p('seq_grip_util_q3_start')),
                 'grip_step':  float(self.p('seq_grip_step_q3')),
                 'avoid': False},
            ]
            if bool(self.p('seq_skip_buffer')):
                # BUFFER 단계 자체를 뺀다 (laps=0 으로 두면 영원히 안 끝난다 —
                # 위 declare_parameter 주석 참고). Q1 → Q2 → Q3 로 바로 이어간다.
                self.seq = [s for s in self.seq if not s['name'].startswith('BUFFER')]
            # map_controller / frenet_planner 가 **항상 떠 있다**고 가정한다
            # (race.launch.py 가 quali 에서 둘 다 띄운다).
            self.map_cli = self.create_client(SetParameters,
                                              '/map_controller/set_parameters')
            self.fr_cli  = self.create_client(SetParameters,
                                              '/frenet_planner/set_parameters')

        # ── 출발 게이트 상태 ───────────────────────────────────────────
        self.gate = bool(self.p('start_gate'))
        self.started = not self.gate      # 게이트가 꺼져 있으면 즉시 출발
        self._amcl_ok = False             # /amcl_pose 를 받아봤나
        self._amcl_last = None            # (x, y, 공분산 대각합)
        self._prev_btn = 0
        self._go_lock = threading.Lock()
        # ── 출발 준비 판정 ─────────────────────────────────────────────
        # "노드가 떴다" 와 "출발해도 된다" 는 다르다. 셋이 다 되어야 준비 완료:
        #   1) auto_localize 가 스캔을 맵에 맞춰 /initialpose 를 쐈나
        #   2) 그 결과로 AMCL 이 /amcl_pose 를 내고 있나
        #   3) 컨트롤러가 살아서 /drive 를 발행하고 있나
        # 하나라도 빠진 채 출발하면 초반 몇 미터를 틀린 위치로 달린다.
        self._fit = None                  # 정합 평균오차 (m)
        self._drive_ok = False            # 컨트롤러가 /drive 를 내고 있나
        self._ready = False               # 준비 완료 배너를 띄웠나
        self._force_t = None              # 미준비 상태에서 버튼 누른 시각

        # ── 수동 개입 ──────────────────────────────────────────────────
        # 조이스틱 LB 로 사람이 조종을 가져가면 컨트롤러를 멈춘다.
        # 나머지 노드(AMCL·LiDAR·맵)는 그대로 두므로 위치 추정이 끊기지 않고,
        # 다시 자율로 넘기면 그 자리에서 이어서 달린다.
        self.manual = False
        self.create_subscription(
            Bool, '/race/manual', self.manual_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.set_enabled(self.started)
        self.create_timer(0.5, self.check_time)
        self.create_timer(0.5, self._republish_enabled)
        self.create_timer(2.0, self._progress_banner)

        if self.gate:
            latched_in = QoSProfile(depth=1,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Joy, '/joy', self.joy_cb, 10)
            self.create_subscription(Bool, '/race/go', self.go_cb, 10)
            self.create_subscription(Float32, '/race/localized',
                                     self.fit_cb, latched_in)
            self.create_subscription(AckermannDriveStamped, '/drive',
                                     self.drive_cb, 10)
            self.create_timer(2.0, self._wait_banner)
            if bool(self.p('start_key')):
                threading.Thread(target=self._key_thread, daemon=True).start()
            self.get_logger().info('초기화 중 — 위치 정렬을 기다립니다…')
        self.get_logger().info(
            f"MissionManager: laps={self.p('laps')}, "
            f"time_limit={self.p('time_limit_sec')}s, "
            f"start_gate={'ON' if self.gate else 'off'}")

    # ── 출발 게이트 ────────────────────────────────────────────────────────

    def _missing(self):
        """아직 안 된 준비 항목 목록."""
        m = []
        if self._fit is None:
            m.append('위치 정렬(auto_localize)')
        if not self._amcl_ok:
            m.append('AMCL pose')
        if not self._drive_ok:
            m.append('컨트롤러(/drive)')
        return m

    def _wait_banner(self):
        """준비될 때까지 남은 항목을 2초마다 보여주고, 다 되면 한 번만 크게 알린다."""
        if self.started:
            return
        missing = self._missing()
        if missing:
            self.get_logger().info(f'초기화 중 — 남은 항목: {", ".join(missing)}')
            return
        if not self._ready:
            self._ready = True
            x, y, cov = self._amcl_last
            btn = self.p('start_button')
            self.get_logger().info(
                '\n'
                '╔══════════════════════════════════════════════════════╗\n'
                '║              출  발  준  비  완  료                  ║\n'
                '╠══════════════════════════════════════════════════════╣\n'
                f'║  위치 정합  평균오차 {self._fit*100:5.1f} cm                      ║\n'
                f'║  AMCL       ({x:+.2f}, {y:+.2f})  공분산합 {cov:.3f}          ║\n'
                '║  컨트롤러   OK                                       ║\n'
                '╠══════════════════════════════════════════════════════╣\n'
                f'║  조이스틱 버튼{btn} 을 누르면 출발합니다              ║\n'
                '╚══════════════════════════════════════════════════════╝')
        else:
            x, y, cov = self._amcl_last
            self.get_logger().info(
                f'[출발 대기] 정합 {self._fit*100:.1f}cm | AMCL ({x:+.2f}, {y:+.2f}) '
                f'공분산합 {cov:.3f} — 버튼을 누르세요')

    def manual_cb(self, msg: Bool):
        if bool(msg.data) == self.manual:
            return
        self.manual = bool(msg.data)
        if self.manual:
            self.get_logger().warn('수동 개입 — 컨트롤러 정지 (/race/enabled=false)')
            self.set_enabled(False)
        elif self.finished:
            self.get_logger().info('자율 복귀 요청 — 이미 완주/정지 상태라 무시')
        elif self.started:
            self.get_logger().info('자율 복귀 — 현재 위치에서 주행을 이어갑니다')
            self.set_enabled(True)
        else:
            self.get_logger().info('자율 복귀 — 아직 출발 전이라 게이트에서 대기')

    def fit_cb(self, msg: Float32):
        self._fit = float(msg.data)
        self.get_logger().info(f'위치 정렬 완료 — 평균오차 {self._fit*100:.1f}cm')

    def drive_cb(self, msg: AckermannDriveStamped):
        self._drive_ok = True

    def _key_thread(self):
        """터미널 Enter 대기.

        ros2 launch 로 띄운 노드는 stdin 이 터미널이 아니라서 input() 이 안 된다.
        제어 터미널이 살아 있으면 /dev/tty 는 열리므로 그쪽을 직접 읽는다.
        (tmux·nohup 등 tty 가 없으면 조용히 포기하고 조이스틱/`race.sh go` 로 간다)
        """
        try:
            with open('/dev/tty') as tty:
                for _ in tty:
                    self.go('Enter 키')
                    return
        except Exception as e:
            self.get_logger().info(
                f'터미널 Enter 대기 불가 ({type(e).__name__}) — '
                '조이스틱 버튼이나 ./race.sh go 를 쓰세요')

    def joy_cb(self, msg: Joy):
        i = int(self.p('start_button'))
        if i < len(msg.buttons):
            btn = msg.buttons[i]
            if btn and not self._prev_btn:      # 눌린 순간에만 (엣지)
                self.go(f'조이스틱 버튼{i}')
            self._prev_btn = btn

    def go_cb(self, msg: Bool):
        if msg.data:
            self.go('./race.sh go')

    def go(self, src: str):
        # 준비가 안 됐으면 한 번은 막는다 — 무엇이 빠졌는지 알려주는 게 목적이다.
        # 다만 대회에서 못 나가는 것이 최악이므로, 3초 안에 다시 누르면 강행한다.
        missing = self._missing()
        if missing and not self.started:
            now = self.get_clock().now()
            recent = (self._force_t is not None
                      and (now - self._force_t).nanoseconds * 1e-9 < 3.0)
            if not recent:
                self._force_t = now
                self.get_logger().warn(
                    f'⚠ 아직 준비 안 됨: {", ".join(missing)} — '
                    '그래도 출발하려면 3초 안에 한 번 더 누르세요')
                return
            self.get_logger().warn(f'⚠ 강제 출발 ({", ".join(missing)} 미완료)')

        with self._go_lock:
            if self.started:
                return
            self.started = True
        # 랩타임·제한시간은 기동 시각이 아니라 **출발 시각**부터 센다
        self.start_time = self.get_clock().now()
        self.lap_start = self.start_time
        self.stage_start = self.start_time   # 단계 제한시간 기준 (lap_start 와 별개)
        if self.seq_enabled and self.seq:
            self.stage_idx = 0
            stage = self.seq[0]
            self.get_logger().info(
                f'════════ 예선 시퀀스 — {stage["name"]} '
                f'(1/{len(self.seq)}, 총 제한 '
                f'{float(self.p("seq_total_limit")):.0f}s) ════════')
            self._enter_stage(stage, self.start_time)
        if not self._amcl_ok:
            self.get_logger().warn(
                '⚠ AMCL pose 미수신 상태로 출발합니다 — init_pose 가정으로 달립니다')
        self.get_logger().info(f'════════ 출발! ({src}) ════════')
        if self.manual:
            self.get_logger().warn(
                '수동 개입 중이라 컨트롤러는 아직 멈춰 있습니다 — '
                'LB 로 자율로 되돌리면 주행을 시작합니다')
            return
        self.set_enabled(True)

    def p(self, name):
        return self.get_parameter(name).value

    def set_enabled(self, val: bool):
        self._enabled_state = val
        self.enabled_pub.publish(Bool(data=val))

    def _republish_enabled(self):
        """현재 상태를 주기적으로 다시 쏜다 (0.5s).

        구독자는 래치로 받게 고쳤지만, 그래도 한 겹 더 둔다 — 컨트롤러가
        늦게 뜨거나 재시작해도 몇백 ms 안에 올바른 상태로 수렴한다.
        상태를 새로 만드는 게 아니라 이미 정해진 값을 반복할 뿐이라 부작용이 없다.
        """
        self.enabled_pub.publish(Bool(data=getattr(self, '_enabled_state', False)))

    def check_time(self):
        tl = self.p('time_limit_sec')
        if self.finished or not self.started:
            return
        if self.seq_enabled and self.seq:
            now = self.get_clock().now()
            # ★ 총 제한(4분)은 **어느 단계에 있든** 지킨다. Q1/Q2 가 예상보다
            #   오래 걸려 Q3 에 닿기도 전에 4분을 넘길 수 있는데, 그때도
            #   규정 위반이므로 여기서 끊는다.
            tot = float(self.p('seq_total_limit'))
            if tot > 0:
                el_all = (now - self.start_time).nanoseconds * 1e-9
                if el_all >= tot:
                    self.get_logger().warn(
                        f'⚠ 총 제한 {tot:.0f}s 도달 '
                        f'({self.seq[self.stage_idx]["name"]} 진행 중) → 정지')
                    self.finish()
                    return
            stage = self.seq[self.stage_idx]
            tl = self._stage_limit
            if stage['time_limit'] <= 0:
                return                       # 이 단계는 랩 기준이다
            # ★ lap_start 가 아니라 stage_start 를 쓴다 — lap_start 는 랩이 끝날
            #   때마다 갱신돼서, Q3 처럼 시간이 기준인 단계에서 그걸 쓰면 랩이
            #   끝날 때마다 제한시간이 다시 시작된다.
            elapsed = (now - self.stage_start).nanoseconds * 1e-9
            if elapsed >= tl:
                self.get_logger().info(f'[{stage["name"]}] 제한시간 {tl:.0f}s 도달')
                self._advance_stage()
            return
        if tl <= 0:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed >= tl:
            self.get_logger().info(f'제한시간 {tl}s 도달 → 정지')
            self.finish()

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        # 대기 중에도 AMCL 상태는 추적한다 (출발 판단 근거)
        c = msg.pose.covariance
        px, py = msg.pose.pose.position.x, msg.pose.pose.position.y
        self._amcl_ok = True
        self._amcl_last = (px, py, float(c[0] + c[7] + c[35]))  # x, y, yaw 분산 합
        if self.finished or not self.started:
            return

        # 출발선 반경 재진입 = 1랩.
        #
        # ★ start_radius 는 **레이싱라인이 원을 실제로 관통**하도록 잡아야 한다.
        #   2026-08-22 quali 주행에서 7바퀴를 돌고 1랩만 세었다. 원인은 반경
        #   1.0m 인데 레이싱라인의 출발선까지 최소거리가 1.142m 여서 **라인이
        #   원에 아예 안 들어왔기** 때문이다. 차가 라인에서 벗어난 순간에만
        #   원을 스쳤고(7바퀴 통틀어 표본 12개), 재진입 프레임이 통째로
        #   빠져버렸다. 1.8m 면 라인이 2.30m 길이로 관통한다(3.5㎧ 에서 0.66초,
        #   /amcl_pose 30Hz 기준 약 20 표본).
        #   ※ 맵/라인을 바꾸면 이 값을 다시 확인할 것 — 너무 키우면 반대편
        #     직선이 원에 걸려 가짜 랩이 센다. 0821_clean 은 2.5m 까지 안전.
        dx, dy = px - self.p('start_x'), py - self.p('start_y')
        inside_now = math.hypot(dx, dy) < self.p('start_radius')
        if not inside_now:
            self.armed = True
        done = inside_now and not self.inside and self.armed
        self.inside = inside_now

        if not done:
            return
        self.lap += 1
        now = self.get_clock().now()
        lap_time = (now - self.lap_start).nanoseconds * 1e-9
        self.lap_start = now
        self.lap_pub.publish(Int32(data=self.lap))
        self.get_logger().info(
            f'{self._stage_tag(self.lap)}LAP {self.lap} 완료! ({lap_time:.2f}s)')
        if self.seq_enabled and self.seq:
            stage = self.seq[self.stage_idx]
            if stage['laps'] > 0 and self.lap >= stage['laps']:
                self._advance_stage()
            else:
                # 랩 기준 단계가 아직 안 끝났거나 시간 기준 단계(Q3)면,
                # 이번 랩을 마친 값으로 속도를 한 칸 올린다.
                self._stage_lap_ramp()
        elif 0 < self.p('laps') <= self.lap:
            self.get_logger().info('목표 랩수 완주 → 정지')
            self.finish()

    # ── 진행 상황 표시 ───────────────────────────────────────────────────

    def _stage_tag(self, n: int = None) -> str:
        """로그 앞에 붙는 `[Q1 1/3] ` 꼬리표. 시퀀스가 아니면 빈 문자열.

        n 을 안 주면 **지금 달리고 있는** 바퀴 번호(= 완주 랩 + 1)를 쓴다.
        출발 직후가 1/3 이고 3바퀴째를 달릴 때 3/3 이다 — 사람이 트랙을
        보면서 세는 방식과 같다. 랩 완주 로그만 방금 끝난 바퀴 번호를
        넘겨 받아 쓴다.
        """
        if not (self.seq_enabled and self.seq):
            return ''
        st = self.seq[self.stage_idx]
        if st['laps'] > 0:
            cur = self.lap + 1 if n is None else n
            return f"[{st['name']} {min(cur, st['laps'])}/{st['laps']}] "
        if st['time_limit'] > 0:
            el = (self.get_clock().now() - self.stage_start).nanoseconds * 1e-9
            return f"[{st['name']} {el:.0f}/{self._stage_limit:.0f}s] "
        return f"[{st['name']}] "

    def _progress_banner(self):
        """주행 중 2초마다 현재 단계와 진행률을 찍는다.

        단계 전환이 로그 한 줄로만 지나가면, 대회장에서 지금 Q 몇을 돌고
        있는지 화면만 보고는 알 수가 없다.
        """
        if self.finished or not self.started:
            return
        if not (self.seq_enabled and self.seq):
            return
        st = self.seq[self.stage_idx]
        el = (self.get_clock().now() - self.lap_start).nanoseconds * 1e-9
        self.get_logger().info(
            f"{self._stage_tag()}단계 {self.stage_idx + 1}/{len(self.seq)}"
            f"  이번 랩 {el:.1f}s"
            f"  회피 {'ON ' if st['avoid'] else 'off'}"
            f"  grip_util {self._grip_now:.2f}"
            + (f" →{st['grip_util']:.2f}"
               if self._grip_now < float(st['grip_util']) - 1e-9 else ''))

    # ── 예선 시퀀스 ──────────────────────────────────────────────────────

    def _stage_time_limit(self, stage, now) -> float:
        """이 단계에 실제로 줄 시간 [s]. 랩 기준이면 0.

        규정이 둘이라 고정으로 못 둔다 — Q3 단독 1분, 전체 4분.
            실제 = min(단계 기본시간, 총제한 − 지금까지 경과)
        Q1~BUFFER2 가 3분 안에 끝났으면 Q3 는 1분을 온전히 쓰고 총 ≤4분,
        3분을 넘겼으면 남은 만큼만 써서 총합이 정확히 4분이 된다.
        """
        base = float(stage['time_limit'])
        if base <= 0:
            return 0.0
        tot = float(self.p('seq_total_limit'))
        if tot <= 0:
            return base
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        return min(base, tot - elapsed)

    def _enter_stage(self, stage, now):
        """단계 진입 공통 처리 — 시간 계산, 로그, 파라미터 전송."""
        self._stage_limit = self._stage_time_limit(stage, now)
        if stage['time_limit'] > 0:
            el = (now - self.start_time).nanoseconds * 1e-9
            if self._stage_limit <= 0.0:
                # 여기 오면 이미 총 제한을 다 썼다. 시간을 0 으로 두면
                # check_time 이 "랩 기준" 으로 오해해 영영 안 끝난다.
                self.get_logger().warn(
                    f'⚠ {stage["name"]} 진입 시점에 이미 총 제한을 다 썼다 '
                    f'(경과 {el:.0f}s) → 즉시 정지')
                self.finish()
                return False
            self.get_logger().info(
                f'  {stage["name"]} 제한시간 {self._stage_limit:.0f}s '
                f'(기본 {stage["time_limit"]}s, 경과 {el:.0f}s, '
                f'총 제한 {float(self.p("seq_total_limit")):.0f}s '
                f'→ 종료 예정 {el + self._stage_limit:.0f}s)')
        self._apply_stage_params(stage)
        return True

    def _advance_stage(self):
        """현재 단계를 끝내고 다음 단계로. 마지막 단계면 진짜로 정지한다."""
        self.stage_idx += 1
        if self.stage_idx >= len(self.seq):
            self.get_logger().info('예선 시퀀스 전체 완료 → 정지')
            self.finish()
            return
        stage = self.seq[self.stage_idx]
        self.lap = 0
        self.armed = False           # 다음 랩 전에 출발선을 한 번 벗어나야 한다
        now = self.get_clock().now()
        self.lap_start = now
        self.stage_start = now
        goal = (f'{stage["laps"]}랩' if stage['laps'] > 0
                else f'{stage["time_limit"]}초')
        self.get_logger().info(
            f'════════ 다음 단계 — {stage["name"]} '
            f'({self.stage_idx + 1}/{len(self.seq)}, {goal}, '
            f'회피 {"ON" if stage["avoid"] else "off"}) ════════')
        self._enter_stage(stage, now)

    def _apply_stage_params(self, stage: dict):
        tgt = float(stage['grip_util'])
        start = float(stage.get('grip_start', 0.0))
        step = float(stage.get('grip_step', 0.0))
        # 램프는 시작값이 목표보다 **낮을 때만** 의미가 있다. 0 이거나 목표
        # 이상이면 그냥 목표값으로 들어간다 (램프 없던 시절과 같은 동작).
        if 0.0 < start < tgt and step > 0.0:
            self._grip_now = start
            n = int(math.ceil((tgt - start) / step)) + 1
            self.get_logger().info(
                f'  {stage["name"]} 속도 램프: {start:.2f} → {tgt:.2f} '
                f'(랩마다 +{step:.2f}, {n}바퀴째에 목표 도달)')
        else:
            self._grip_now = tgt
        self._set_remote_param(self.map_cli, 'grip_util', self._grip_now)
        self._set_remote_param(self.fr_cli, 'avoid_enabled', bool(stage['avoid']))

    def _stage_lap_ramp(self):
        """랩을 하나 돌 때마다 grip_util 을 한 칸 올린다 (Q3 램프).

        랩 완주 **직후**에 부른다. 목표에 닿으면 더 안 올리고 조용히 빠진다.
        """
        if not (self.seq_enabled and self.seq):
            return
        stage = self.seq[self.stage_idx]
        step = float(stage.get('grip_step', 0.0))
        tgt = float(stage['grip_util'])
        if step <= 0.0 or self._grip_now >= tgt - 1e-9:
            return
        self._grip_now = min(tgt, self._grip_now + step)
        at = '목표 도달' if self._grip_now >= tgt - 1e-9 else f'목표 {tgt:.2f}'
        self.get_logger().info(
            f'{self._stage_tag()}속도 램프 grip_util → '
            f'{self._grip_now:.2f} ({at})')
        self._set_remote_param(self.map_cli, 'grip_util', self._grip_now)

    def _set_remote_param(self, cli, name: str, value):
        """다른 노드에 /set_parameters 로 값을 하나 쏜다 (비동기, 응답 대기 안 함).

        ⚠ 응답을 안 기다리므로 단계 전환 직후 한두 틱은 옛 값으로 달릴 수 있다.
          기다리면 이 콜백이 블록돼 랩 카운트가 밀리므로 비동기를 택했다.
        """
        if cli is None:
            return
        if not cli.service_is_ready():
            self.get_logger().warn(
                f'{cli.srv_name} 서비스가 아직 안 떠서 {name} 적용을 건너뜀 '
                f'— 그 노드가 늦게 뜨는지 확인할 것')
            return
        if isinstance(value, bool):
            pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        else:
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                double_value=float(value))
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=pv)]
        cli.call_async(req)
        self.get_logger().info(f'{cli.srv_name} {name} → {value}')

    def finish(self):
        self.finished = True
        self.set_enabled(False)


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C 로 컨텍스트가 이미 닫힌 뒤면 발행이 예외를 던진다.
        # 종료 경로에서만 나는 노이즈라 조건을 두고 조용히 넘긴다.
        if rclpy.ok():
            node.set_enabled(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
