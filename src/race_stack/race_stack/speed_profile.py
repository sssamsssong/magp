#!/usr/bin/env python3
"""speed_profile — 마찰원 결합 forward-backward 속도 프로파일 (컨트롤러 공용).

왜 "상한" 이 아니라 이걸 쓰나
──────────────────────────────
예전에는 컨트롤러마다 `clip(속도, min_speed, max_speed)` 로만 제한했다. 그런데
max_speed 가 CSV 속도보다 낮으면 **전 구간이 그 값으로 잘려 등속 주행**이 된다.
2026-08-09 이전 Stanley 가 정확히 그 상태였다 — q1_max=1 인데 CSV 는 0.75~6.0㎧
라서, 곡률 프로파일이 있으나 마나였고 직선에서도 코너에서도 똑같이 1㎧ 로 돌았다.

속도를 배수로 스케일하는 것도 답이 아니다. 속도를 k 배 하면 요구 횡가속(v²κ)은
**k² 배**가 되어 비선형이고, 어느 코너가 한계인지까지 바뀐다 — 코너에 맞추면
직선이 느리고 직선에 맞추면 코너가 터진다. 어떤 k 로도 동시에 못 맞춘다.

그래서 속도가 아니라 **마찰 예산**을 조절한다:

    a_g = grip_util(η) · μ_eff · g          ← 타이어가 낼 수 있는 총 가속도

v ∝ √η 라 코너는 √η 배로 줄지만, 직선은 종가속과 구간 길이가 지배해 균일하게
안 줄어든다. 그게 물리적으로 맞다.

알고리즘 (Heilmeier / ForzaETH 계열)
────────────────────────────────────
    1) 곡률 한계   v ≤ √(a_g / κ)
    2) 역방향 패스  다음 코너까지 설 수 있게 미리 감속
    3) 전진 패스   낼 수 있는 만큼만 가속
    2·3 은 **마찰원 결합**:   a_lon = a_cap · √(1 − (a_lat/a_g)²)
       → 돌면서 동시에 세게 밟거나 설 수 없다. 코너 진입에서 더 일찍 감속하고
         탈출에서 더 늦게 가속하게 만드는, 순진한 프로파일과의 결정적 차이다.
    a_lat 이 v 에 의존하므로 수렴할 때까지 반복한다.

★ μ_eff 는 mu × pac_D 다 (MAP 논문 III-B: "peak value D̃ is in fact μ·Fz,i·Di").
  mu 혼자가 아니다 — 그대로 넣으면 20% 낙관적인 프로파일이 나온다.
"""
import math

import numpy as np

GRAVITY = 9.80665


def curvature_from_path(x, y):
    """경로점에서 곡률 [1/m]. 닫힌 경로를 가정하지 않는 중심차분."""
    dx, dy = np.gradient(x), np.gradient(y)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    return np.abs(dx * ddy - dy * ddx) / np.maximum((dx ** 2 + dy ** 2) ** 1.5, 1e-9)


def solve_profile(x, y, csv_v=None, *, mu_eff, grip_util, lf, lr, v_min,
                  a_lon_acc=0.0, a_lon_brk=0.0, hard_cap=0.0,
                  curvature=None, iters=5, closed=True):
    """속도 프로파일을 푼다. 반환 (v, info).

    csv_v      웨이포인트에 실린 속도. **상한으로만** 쓴다 — 설계자가 의도적으로
               낮춰둔 구간(피트 진입 등)은 존중하되, 다른 μ 에서 만든 낙관적
               값은 물리가 이긴다. None 이면 순수 물리로만.
    a_lon_acc  가속 상한. 0 이면 후륜구동 가정으로 유도한다 (a_g · lf/L).
               구동륜 하중분율이 한계이기 때문이다.
    a_lon_brk  제동 상한. 0 이면 네 바퀴를 다 쓰므로 예산 전체(a_g).
    hard_cap   0 이면 **끔**. 물리와 무관한 안전 천장이 필요할 때만 쓴다
               (μ 나 η 가 틀려도 이 속도는 안 넘는다는 마지막 방어선).
    v_min      맨 **마지막에** 적용한다. 패스 도중에 올리면 제동 제약이 깨진다.
    """
    x = np.asarray(x, float);  y = np.asarray(y, float)
    n = len(x)
    if n < 3:
        return np.full(max(n, 1), v_min), {}

    if closed:
        ds = np.hypot(np.roll(x, -1) - x, np.roll(y, -1) - y)
    else:
        ds = np.r_[np.hypot(np.diff(x), np.diff(y)), 1e-3]
    ds = np.maximum(ds, 1e-3)

    kap = curvature_from_path(x, y) if curvature is None else np.abs(np.asarray(curvature, float))
    kap = np.maximum(kap, 1e-4)

    eta = float(np.clip(grip_util, 0.05, 1.0))
    a_g = eta * float(mu_eff) * GRAVITY
    L = max(float(lf) + float(lr), 1e-6)
    a_acc = float(a_lon_acc) if a_lon_acc > 0 else a_g * float(lf) / L
    a_brk = float(a_lon_brk) if a_lon_brk > 0 else a_g
    a_acc, a_brk = min(a_acc, a_g), min(a_brk, a_g)

    v = np.sqrt(a_g / kap)
    if csv_v is not None:
        cv = np.asarray(csv_v, float)
        if cv.shape == v.shape and np.all(cv > 0):
            v = np.minimum(v, cv)
    if hard_cap and hard_cap > 0:
        v = np.minimum(v, float(hard_cap))

    def _step(v_from, ki, cap, dsi, inner=4):
        """마찰원 결합 한 스텝을 **음함수로** 푼다.

        구하려는 속도 u=v² 가 자기 자신의 횡가속에 들어가므로 음함수다:
            u = v_from² + 2·ds·cap·√(1 − (u·κ/a_g)²)

        ★ 이걸 명시적으로(현재 v 로 횡가속을 평가) 풀면 안 된다 (2026-08-09).
          곡률 한계에서 출발하면 r = v²κ/a_g 가 정확히 1 이라 a_lon = 0 이 되고,
          그러면 역방향 패스가 v[i] = min(v[i], v[j]) 로 변해 **코너 속도가
          직선까지 그대로 전파**된다. 게다가 패스가 단조 감소(min)라 한 번
          낮아지면 회복되지 않는다. 실제로 CSV 상한을 빼면 최고 속도가
          5.00 → 2.57㎧ 로 **떨어지는** 증상이 나왔다 — 상한을 없앴는데 느려지는
          모순. CSV 가 대부분의 점을 한계 아래로 낮춰준 덕에 가려져 있었다.

        v_from(이웃의 속도)에서 시작해 몇 번 되풀이하면 수렴한다. 시작점이
        한계 아래라 첫 평가에서 여력이 살아 있고, 반복하며 정확한 값으로 내려온다.
        """
        u = v_from * v_from
        for _ in range(inner):
            r = min(u * ki / max(a_g, 1e-9), 1.0)
            u = v_from * v_from + 2.0 * dsi * cap * math.sqrt(max(0.0, 1.0 - r * r))
        return math.sqrt(max(u, 0.0))

    # ★ 구간 곡률은 두 끝의 **큰 쪽**을 쓴다. 한쪽 끝만 보면 곡률이 급히 변하는
    #   구간에서 합성 가속이 마찰원을 벗어난다 (실측 5.8% 초과).
    kseg = np.maximum(kap, np.roll(kap, -1))

    rng = range(n - 1, -1, -1) if closed else range(n - 2, -1, -1)
    for _ in range(int(iters)):
        for i in rng:                                   # 역방향 — 감속
            j = (i + 1) % n
            v[i] = min(v[i], _step(v[j], kseg[i], a_brk, ds[i]))
        for i in range(n if closed else n - 1):         # 전진 — 가속
            j = (i + 1) % n
            v[j] = min(v[j], _step(v[i], kseg[i], a_acc, ds[i]))
        if hard_cap and hard_cap > 0:
            v = np.minimum(v, float(hard_cap))

    v = np.maximum(v, float(v_min))
    a_lat = v * v * kap
    info = dict(eta=eta, a_g=a_g, a_acc=a_acc, a_brk=a_brk,
                v_min=float(v.min()), v_max=float(v.max()),
                v_med=float(np.median(v)), a_lat_max=float(a_lat.max()),
                lap_time=float(np.sum(ds / np.maximum(v, 1e-6))),
                capped=float(np.mean(v >= hard_cap - 1e-6)) if hard_cap else 0.0)
    return v, info


def steer_speed_limit(delta, wheelbase, mu_eff, grip_util):
    """반응형 컨트롤러용 — **지령 조향각**에서 물리 속도 한계를 낸다.

    레이싱라인이 없는 gap_follower 같은 컨트롤러는 경로 곡률을 모른다. 대신
    자기가 내리려는 조향각이 곧 곡률이다:  κ = |tan δ| / L.
    그러면 그 곡률을 마찰 예산 안에서 돌 수 있는 최대 속도가 정해진다.

        v ≤ √(η·μ_eff·g / κ)

    "조향이 크면 0.6배" 같은 임의 계수보다 이쪽이 옳다 — 노면(μ)과 차량 기하가
    바뀌면 자동으로 따라간다.
    """
    kap = abs(math.tan(float(delta))) / max(float(wheelbase), 1e-6)
    a_g = float(np.clip(grip_util, 0.05, 1.0)) * float(mu_eff) * GRAVITY
    return math.sqrt(a_g / max(kap, 1e-4))
