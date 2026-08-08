#!/usr/bin/env python3
"""MAP 컨트롤러 파라미터 최적화 — Pacejka 자전거 모델 기반.

Optuna TPE + vehicle_sim.py 직접 적분 (ROS / gym 불필요).
race_script.yaml 의 map: 섹션을 읽어 고정 파라미터로 사용하고,
최적화 결과를 같은 파일에 반영한다.

실행:
  python3 src/race_stack/race_stack/map_optimize.py [--trials N] [--timeout S]

최적화 대상:
  l1_min      최소 L1 lookahead [m]    — 직선 진동의 주 원인
  l1_k        속도 비례 lookahead 게인  — L1 = max(l1_min, l1_k × v)
  steer_alpha 조향 저역통과 필터 계수   — 낮을수록 빠르지만 진동
  speed_scale 웨이포인트 속도 배율
  a_brake     코너 전 제동 가속도 [m/s²]

목표 함수 (낮을수록 좋음):
  score = mean_CTE × 10
        + steer_rate_std × 20    (직선 진동 패널티)
        + lap_time × 0.5         (완주 시)
        + PENALTY                (미완주 시 — 진행 거리로 부분 점수)
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys

import numpy as np
import optuna
import yaml

optuna.logging.set_verbosity(optuna.logging.WARNING)

# ── 경로 설정 ──────────────────────────────────────────────────────────────
_HERE      = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
_RACE_SCRIPT = os.path.join(_REPO_ROOT, 'race_script.yaml')

sys.path.insert(0, _HERE)
from vehicle_sim import VehicleParams, bicycle_ode

# ── 시뮬 상수 ──────────────────────────────────────────────────────────────
G        = 9.81
DT       = 0.005          # 적분 스텝 [s]
CTRL_HZ  = 50             # 제어 주파수 [Hz]
CTRL_DT  = 1.0 / CTRL_HZ # 제어 주기
STEPS_PER_CTRL = max(1, round(CTRL_DT / DT))  # 적분 스텝 수 / 제어 주기

TIMEOUT   = 40.0   # 최대 시뮬 시간 [s]
PENALTY   = 5000.0

DEPART_DIST = 3.0  # 출발 확인 거리 [m]
ARRIVE_DIST = 1.5  # 도착 인식 거리 [m] (출발점으로 귀환)

MAP_OUT_DIST = 5.0  # 맵 이탈 판정 [m] (웨이포인트에서 멀리)


# ── 유틸 ──────────────────────────────────────────────────────────────────

def _rk4_step(state: np.ndarray, delta: float, Fx: float,
              p: VehicleParams) -> np.ndarray:
    def f(s):
        return bicycle_ode(0.0, s, delta, Fx, p)
    k1 = np.array(f(state))
    k2 = np.array(f(state + DT / 2 * k1))
    k3 = np.array(f(state + DT / 2 * k2))
    k4 = np.array(f(state + DT * k3))
    return state + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def _load_waypoints(path: str) -> np.ndarray:
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append([float(r['x']), float(r['y']), float(r['speed'])])
    wp = np.array(rows)
    # 연속 중복 포인트 제거 (arc length=0 → CubicSpline 오류 방지)
    keep = np.concatenate([[True], np.linalg.norm(np.diff(wp[:, :2], axis=0), axis=1) > 1e-6])
    return wp[keep]


def _precompute_kappa(wp: np.ndarray) -> np.ndarray:
    """각 웨이포인트의 곡률 [1/m]."""
    from scipy.interpolate import CubicSpline
    pts = wp[:, :2]
    # 마지막 포인트가 첫 포인트와 동일한 폐루프면 제거
    if np.linalg.norm(pts[-1] - pts[0]) < 1e-6:
        pts = pts[:-1]
    n = len(pts)
    ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(ds)])
    gap = np.linalg.norm(pts[0] - pts[-1])
    s_ext = np.append(s, s[-1] + max(gap, 1e-6))
    x_ext = np.append(pts[:, 0], pts[0, 0])
    y_ext = np.append(pts[:, 1], pts[0, 1])
    cs_x = CubicSpline(s_ext, x_ext, bc_type='periodic')
    cs_y = CubicSpline(s_ext, y_ext, bc_type='periodic')
    dx, dy   = cs_x(s, 1), cs_y(s, 1)
    ddx, ddy = cs_x(s, 2), cs_y(s, 2)
    denom = np.maximum((dx**2 + dy**2) ** 1.5, 1e-6)
    return np.abs(dx * ddy - dy * ddx) / denom


def _build_speed_profile(wp: np.ndarray, a_brake: float,
                         speed_scale: float) -> np.ndarray:
    n = len(wp)
    v = wp[:, 2] * speed_scale
    ds = np.array([
        np.linalg.norm(wp[(i + 1) % n, :2] - wp[i, :2])
        for i in range(n)
    ])
    for _ in range(3):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2.0 * a_brake * ds[i]))
    return v


def _find_nearest(wp: np.ndarray, pos: np.ndarray, hint: int,
                  search: int = 60) -> int:
    n = len(wp)
    idxs = [(hint + i) % n for i in range(-5, search)]
    dists = [np.linalg.norm(wp[i, :2] - pos) for i in idxs]
    return idxs[int(np.argmin(dists))]


def _find_lookahead(wp: np.ndarray, pos: np.ndarray, L1: float,
                    near: int) -> np.ndarray | None:
    n = len(wp)
    arc = 0.0
    for i in range(n):
        i0 = (near + i) % n
        i1 = (near + i + 1) % n
        seg = float(np.linalg.norm(wp[i1, :2] - wp[i0, :2]))
        if arc + seg >= L1:
            t = float(np.clip((L1 - arc) / max(seg, 1e-6), 0.0, 1.0))
            return (1.0 - t) * wp[i0, :2] + t * wp[i1, :2]
        arc += seg
    return None


# ── Pacejka 역함수 LUT 빌드 ────────────────────────────────────────────────

def _build_pac_lut(p: VehicleParams):
    alphas = np.linspace(-0.5, 0.5, 1000)
    B, C, D, E = p.pac_B, p.pac_C, p.pac_D, p.pac_E
    x  = B * alphas
    Fy = D * np.sin(C * np.arctan(x - E * (x - np.arctan(x))))
    return alphas, Fy


def _pac_inv(Fy_req: float, alpha_lut: np.ndarray, Fy_lut: np.ndarray) -> float:
    clamped = float(np.clip(Fy_req, Fy_lut[0] * 0.99, Fy_lut[-1] * 0.99))
    return float(np.interp(clamped, Fy_lut, alpha_lut))


# ── 한 에피소드 시뮬레이션 ─────────────────────────────────────────────────

def run_episode(wp: np.ndarray, v_profile: np.ndarray, kappa: np.ndarray,
                p: VehicleParams, params: dict) -> float:
    """MAP 컨트롤러 + 자전거 ODE 폐루프 시뮬.

    Returns
    -------
    score : float — 낮을수록 좋음.
    """
    l1_min      = params['l1_min']
    l1_k        = params['l1_k']
    steer_alpha = params['steer_alpha']

    alpha_lut, Fy_lut = _build_pac_lut(p)

    n   = len(wp)
    wb  = p.lf + p.lr
    tau = 0.15   # 속도 응답 시정수

    # 초기 상태
    yaw0  = math.atan2(wp[1, 1] - wp[0, 1], wp[1, 0] - wp[0, 0])
    state = np.array([0.0, 0.0, 0.0, float(wp[0, 0]), float(wp[0, 1]), yaw0])
    start_xy  = wp[0, :2].copy()

    steer_cmd = 0.0
    v_cmd     = 0.0
    near_idx  = 0
    elapsed   = 0.0
    departed  = False

    cte_list        = []
    steer_rate_list = []
    prev_steer_cmd  = 0.0

    ctrl_tick = 0

    while elapsed < TIMEOUT:
        # ── 제어 계산 (CTRL_HZ) ───────────────────────────────────────────
        if ctrl_tick % STEPS_PER_CTRL == 0:
            vx, vy, r, x, y, psi = state
            v    = max(float(np.hypot(vx, vy)), 0.3)
            pos  = np.array([x, y])

            near_idx = _find_nearest(wp, pos, near_idx)
            L1       = max(l1_min, l1_k * v)
            goal     = _find_lookahead(wp, pos, L1, near_idx)

            if goal is None:
                steer_cmd = 0.0
                v_cmd     = v_profile[near_idx]
            else:
                dx  = goal[0] - x
                dy  = goal[1] - y
                bearing = math.atan2(dy, dx)
                eta = math.atan2(math.sin(bearing - psi), math.cos(bearing - psi))

                a_lat = 2.0 * v * v * math.sin(eta) / max(L1, 1e-3)
                Fzf   = p.m * G * p.lr / wb
                F_yf  = p.m * a_lat * p.lr / wb
                alpha_f = _pac_inv(F_yf / (p.mu * Fzf), alpha_lut, Fy_lut)

                delta_kin = math.atan(wb * 2.0 * math.sin(eta) / max(L1, 1e-3))
                raw = float(np.clip(delta_kin + alpha_f, -0.4189, 0.4189))
                steer_cmd = steer_alpha * steer_cmd + (1.0 - steer_alpha) * raw
                v_cmd     = float(v_profile[near_idx])

            # CTE (nearest waypoint까지 거리)
            cte = float(np.linalg.norm(wp[near_idx, :2] - np.array([x, y])))
            cte_list.append(cte)

            steer_rate_list.append(abs(steer_cmd - prev_steer_cmd) / CTRL_DT)
            prev_steer_cmd = steer_cmd

            # 맵 이탈 판정
            if cte > MAP_OUT_DIST:
                n_done = ctrl_tick
                progress = min(near_idx / n, 0.99)
                return PENALTY * (1.0 - progress) + PENALTY

        # ── 적분 ─────────────────────────────────────────────────────────
        vx = state[0]
        dv = (v_cmd - vx) / tau
        a_max   = 9.51
        a_brake = params.get('a_brake', 6.09)
        Fx = p.m * max(-a_brake, min(a_max, dv))
        state = _rk4_step(state, steer_cmd, Fx, p)
        state[0] = max(0.0, state[0])

        elapsed   += DT
        ctrl_tick += 1

        # ── 완주 판정 ─────────────────────────────────────────────────────
        pos = np.array([state[3], state[4]])
        dist_from_start = float(np.linalg.norm(pos - start_xy))
        if dist_from_start > DEPART_DIST:
            departed = True
        if departed and dist_from_start < ARRIVE_DIST:
            # 완주
            mean_cte       = float(np.mean(cte_list)) if cte_list else 99.0
            steer_rate_std = float(np.std(steer_rate_list)) if steer_rate_list else 99.0
            return elapsed * 0.5 + mean_cte * 10.0 + steer_rate_std * 20.0

    # 타임아웃
    progress = min(near_idx / n, 0.99)
    return PENALTY * (1.0 - progress) + PENALTY


# ── Optuna 최적화 ──────────────────────────────────────────────────────────

def optimize(wp: np.ndarray, kappa: np.ndarray, p: VehicleParams,
             baseline: dict, n_trials: int, timeout_s: float):
    best_scores = [999999.0]

    def objective(trial: optuna.Trial) -> float:
        params = {
            'l1_min':      trial.suggest_float('l1_min',      0.40, 2.50),
            'l1_k':        trial.suggest_float('l1_k',        0.15, 0.80),
            'steer_alpha': trial.suggest_float('steer_alpha', 0.10, 0.90),
            'speed_scale': trial.suggest_float('speed_scale', 0.5,  3.0),
            'a_brake':     trial.suggest_float('a_brake',     3.0,  12.0),
        }
        v_profile = _build_speed_profile(wp, params['a_brake'], params['speed_scale'])
        score = run_episode(wp, v_profile, kappa, p, params)

        tag = '★' if score < best_scores[0] else ' '
        if score < best_scores[0]:
            best_scores[0] = score
        completed = score < PENALTY
        status = f'{score:.1f}' if completed else f'미완주({score:.0f})'
        print(f'  [{trial.number:3d}]{tag} score={status:12s}  '
              f'l1={params["l1_min"]:.2f}/{params["l1_k"]:.2f}  '
              f'α={params["steer_alpha"]:.2f}  '
              f'spd={params["speed_scale"]:.2f}  '
              f'brk={params["a_brake"]:.1f}')
        return score

    sampler = optuna.samplers.TPESampler(seed=42)
    study   = optuna.create_study(direction='minimize', sampler=sampler)
    study.enqueue_trial(baseline)   # 기준값을 첫 번째 시도로 등록

    study.optimize(objective, n_trials=n_trials, timeout=timeout_s)
    return study


# ── race_script.yaml 업데이트 ──────────────────────────────────────────────

def update_race_script(best: dict):
    with open(_RACE_SCRIPT) as f:
        text = f.read()

    keys = {
        'l1_min':      best['l1_min'],
        'l1_k':        best['l1_k'],
        'steer_alpha': best['steer_alpha'],
        'speed_scale': best['speed_scale'],
        'a_brake':     best['a_brake'],
    }
    for key, val in keys.items():
        text = re.sub(
            rf'^(  {key}:)\s+\S+',
            rf'\g<1> {round(val, 4)}',
            text, flags=re.MULTILINE,
        )

    with open(_RACE_SCRIPT, 'w') as f:
        f.write(text)
    print(f'  → {_RACE_SCRIPT} 업데이트 완료')


# ── 메인 ──────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='MAP 파라미터 최적화 (Pacejka 자전거 모델)')
    ap.add_argument('--trials',  type=int,   default=200,  help='최대 시도 횟수')
    ap.add_argument('--timeout', type=float, default=600,  help='최대 시간 [s]')
    ap.add_argument('--no-write', action='store_true',     help='race_script.yaml 미기록')
    ap.add_argument('--wp',      type=str,   default=None, help='웨이포인트 CSV 경로')
    args = ap.parse_args()

    # ── race_script.yaml 로드 ─────────────────────────────────────────────
    with open(_RACE_SCRIPT) as f:
        cfg = yaml.safe_load(f)

    mc = cfg.get('map', {})
    p  = VehicleParams(
        m     = float(mc.get('mass',  3.354)),
        lf    = float(mc.get('lf',    0.1757)),
        lr    = float(mc.get('lr',    0.1441)),
        Iz    = 0.050,
        mu    = float(mc.get('mu',    0.40)),
        pac_B = float(mc.get('pac_b', 3.8)),
        pac_C = float(mc.get('pac_c', 1.3)),
        pac_D = float(mc.get('pac_d', 1.15)),
        pac_E = float(mc.get('pac_e', -0.5)),
    )

    baseline = {
        'l1_min':      float(mc.get('l1_min',      0.90)),
        'l1_k':        float(mc.get('l1_k',        0.35)),
        'steer_alpha': float(mc.get('steer_alpha',  0.1981)),
        'speed_scale': float(mc.get('speed_scale',  2.0)),
        'a_brake':     float(mc.get('a_brake',      6.0902)),
    }

    # ── 웨이포인트 로드 ───────────────────────────────────────────────────
    if args.wp:
        wp_path = args.wp
    else:
        _ws  = os.environ.get('RACE_WS', os.path.expanduser('~/f1tenth_ws'))
        _src = os.path.join(_ws, 'src', 'race_stack')
        if not os.path.isdir(os.path.join(_src, 'waypoints')):
            _src = os.path.join(_REPO_ROOT, 'src', 'race_stack')
        wp_path = os.path.join(_src, 'waypoints', cfg['waypoints'] + '.csv')

    if not os.path.exists(wp_path):
        print(f'웨이포인트 없음: {wp_path}')
        sys.exit(1)

    wp = _load_waypoints(wp_path)
    kappa = _precompute_kappa(wp)

    print('=' * 70)
    print(' MAP 파라미터 최적화 (Pacejka 자전거 모델)')
    print('=' * 70)
    print(f'  웨이포인트 : {wp_path}  ({len(wp)}개)')
    print(f'  μ={p.mu}  B={p.pac_B} C={p.pac_C} D={p.pac_D} E={p.pac_E}')
    print(f'  m={p.m}kg  lf={p.lf}  lr={p.lr}')
    print(f'  trials={args.trials}  timeout={args.timeout}s')
    print()
    print('  기준값:')
    for k, v in baseline.items():
        print(f'    {k:12s} = {v}')
    print()

    # 기준값 평가
    v_profile_base = _build_speed_profile(wp, baseline['a_brake'], baseline['speed_scale'])
    score_base = run_episode(wp, v_profile_base, kappa, p, baseline)
    completed_base = score_base < PENALTY
    print(f'  기준값 score: {score_base:.2f}  '
          f'({"완주" if completed_base else "미완주"})')
    print()
    print(f'  {"#":>3}  {"score":>12}  {"l1_min/k":>10}  '
          f'{"α":>5}  {"spd":>5}  {"brk":>5}')
    print('-' * 70)

    study = optimize(wp, kappa, p, baseline, args.trials, args.timeout)

    best   = study.best_params
    best_t = study.best_value
    n_done = len(study.trials)

    print('\n' + '=' * 70)
    print(f' 완료: {n_done}회 시도')
    print('=' * 70)
    print(f'  기준 score : {score_base:.2f}')
    print(f'  최적 score : {best_t:.2f}  ({"완주" if best_t < PENALTY else "미완주"})')
    print()
    print('  최적 파라미터:')
    for k, v in best.items():
        old = baseline.get(k, '?')
        print(f'    {k:12s}: {old} → {v:.4f}')

    if not args.no_write:
        print()
        update_race_script(best)
    else:
        print('\n  (--no-write 지정: race_script.yaml 미기록)')


if __name__ == '__main__':
    main()
