#!/usr/bin/env python3
"""joy_diag.py — 트리거 입력과 실제 나가는 속도 명령을 나란히 놓고 본다.

  ./race.sh joy 가 돌고 있는 상태에서, **바퀴를 띄우고**
  RT 를 절반쯤 당겨 8초 내내 그대로 유지하며 실행한다.

  핵심 판정: "RT 를 당기고 있는데 /teleop 속도가 0" 인 프레임이 있는가.
  있으면 소프트웨어(끊김), 없으면 명령은 멀쩡하고 원인은 VESC·모터 쪽이다.
"""
import os
import time

import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

DUR = 8.0
BUCKET = 0.25

cfg = {}
for path in (os.environ.get('RACE_VEHICLE_OVERLAY', ''),
             os.environ.get('RACE_SCRIPT', '')):
    if path and os.path.exists(path):
        with open(path) as f:
            cfg = ((yaml.safe_load(f) or {}).get('joystick') or {}) or cfg
        if cfg:
            break
AX_SPEED = int(cfg.get('axis_speed', 5))
BTN_REV = int(cfg.get('btn_reverse', 5))

rclpy.init()
n = Node('joy_diag')
joy, tel, erpm = [], [], []
n.create_subscription(Joy, '/joy',
                      lambda m: joy.append((time.time(), list(m.axes), list(m.buttons))), 10)
n.create_subscription(AckermannDriveStamped, '/teleop',
                      lambda m: tel.append((time.time(), m.drive.speed)), 10)
n.create_subscription(Float64, '/commands/motor/speed',
                      lambda m: erpm.append((time.time(), m.data)), 10)

print(f'>>> RT 를 절반쯤 당겨 {DUR:.0f}초 내내 유지하세요 (다른 버튼 만지지 말 것)...')
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)

if not joy or not tel:
    print('데이터 없음 — ./race.sh joy 가 돌고 있는지 확인하세요.')
    raise SystemExit

# RT 원시값 → 0~1 (휴지 +1, 당김 -1 규약 기준. 반대면 자동으로 뒤집는다)
raw = [(t, a[AX_SPEED]) for t, a, _ in joy if len(a) > AX_SPEED]
lo, hi = min(v for _, v in raw), max(v for _, v in raw)
rest = hi if abs(hi) > abs(lo) else lo


def pull(v):
    return abs(v - rest) / 2.0


# /teleop 각 프레임을, 직전 /joy 프레임과 짝짓는다
pairs = []
i = 0
for t, sp in tel:
    while i + 1 < len(raw) and raw[i + 1][0] <= t:
        i += 1
    pairs.append((t - t0, pull(raw[i][1]), sp))

held = [(u, sp) for _, u, sp in pairs if u > 0.10]
dead = [1 for u, sp in held if abs(sp) < 1e-6]

print(f'\n/joy {len(joy)/DUR:.1f}Hz   /teleop {len(tel)/DUR:.1f}Hz   '
      f'/commands/motor/speed {len(erpm)/DUR:.1f}Hz')
print(f'RT axis[{AX_SPEED}] 범위 {lo:+.3f} ~ {hi:+.3f}  (휴지값 {rest:+.1f} 로 판정)')
print(f'RB btn[{BTN_REV}] 눌린 프레임 '
      f'{sum(b[BTN_REV] for _, _, b in joy if len(b) > BTN_REV)}/{len(joy)}')

print('\n시간   RT당김   속도명령')
for k in range(int(DUR / BUCKET)):
    seg = [(u, sp) for dt, u, sp in pairs if k * BUCKET <= dt < (k + 1) * BUCKET]
    if not seg:
        continue
    u = sum(x for x, _ in seg) / len(seg)
    sp = sum(x for _, x in seg) / len(seg)
    bar = '█' * int(u * 20)
    print(f'{k*BUCKET:4.1f}s  {u*100:4.0f}% {bar:<20s} {sp:+5.2f} m/s')

print()
if not held:
    print('※ RT 를 당긴 프레임이 없습니다 — 당긴 채로 다시 실행하세요.')
elif dead:
    print(f'★ RT 를 당기고 있는데 속도 0 인 프레임 {len(dead)}/{len(held)}')
    print('  → 명령이 끊깁니다. 소프트웨어(워치독/캘리) 문제입니다.')
else:
    print(f'★ RT 를 당긴 {len(held)}프레임 전부 속도가 실려 나갔습니다 — 명령은 멀쩡합니다.')
    print('  그런데도 차가 퉁퉁대면 원인은 VESC·모터 쪽입니다')
    print('  (수동 조종엔 가속 제한이 없어, 정지에서 급지령하면 센서리스 BLDC 가 떱니다).')
rclpy.shutdown()
