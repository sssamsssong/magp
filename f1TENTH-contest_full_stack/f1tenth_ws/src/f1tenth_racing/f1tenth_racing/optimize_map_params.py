#!/usr/bin/env python3
"""
MAP 컨트롤러 파라미터 자동 최적화
Optuna (TPE Bayesian) + f1tenth_gym 직접 구동
튜닝 대상: L1_MIN, L1_K, STEER_ALPHA, A_BRAKE, PAC_D
차량 물리값(MASS, MU, LF, LR, WHEELBASE)은 고정
"""

import numpy as np
import gym
import optuna
import csv
import re
import os
from scipy.interpolate import CubicSpline
from f110_gym.envs.base_classes import Integrator

optuna.logging.set_verbosity(optuna.logging.WARNING)

_THIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
MAP_PATH   = os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps', 'sohot_clean')
WP_PATH    = os.path.join(_REPO_ROOT, '..', 'src', 'race_stack', 'waypoints', 'sohot_clean_wp.csv')
MAP_NODE_PATH = os.path.join(_THIS_DIR, 'map_controller_node.py')

TIMEOUT    = 60.0
PENALTY    = 10000.0   # 완주 못했을 때 기본 패널티
N_WP       = 417       # 웨이포인트 수 (sohot_clean)
CTRL_STEPS = 3         # gym dt=0.01, 제어 33Hz

# 고정 차량 물리값
WHEELBASE = 0.33
MAX_STEER = 0.4189   # gym s_max
MASS      = 3.74
LF        = 0.15875
LR        = 0.17145
MU        = 1.0489
PAC_B     = 3.629    # gym C_Sf=4.718 등가: BCD = C_Sf → B = 4.718/(1.3*1.0)
PAC_C     = 1.3
PAC_E     = -0.5
A_BRAKE_DEFAULT = 9.4675


def load_waypoints(path):
    pts = []
    with open(path) as f:
        for row in csv.DictReader(f):
            pts.append([float(row['x']), float(row['y']), float(row['speed'])])
    return np.array(pts)


def precompute_curvature(wp):
    from scipy.interpolate import CubicSpline
    pts = wp[:, :2]
    n   = len(pts)
    diffs = np.diff(pts, axis=0)
    ds    = np.linalg.norm(diffs, axis=1)
    s     = np.concatenate([[0.0], np.cumsum(ds)])
    s_ext = np.append(s, s[-1] + np.linalg.norm(pts[0] - pts[-1]))
    x_ext = np.append(pts[:, 0], pts[0, 0])
    y_ext = np.append(pts[:, 1], pts[0, 1])
    cs_x  = CubicSpline(s_ext, x_ext, bc_type='periodic')
    cs_y  = CubicSpline(s_ext, y_ext, bc_type='periodic')
    dx  = cs_x(s, 1);  dy  = cs_y(s, 1)
    ddx = cs_x(s, 2);  ddy = cs_y(s, 2)
    denom = np.maximum((dx**2 + dy**2) ** 1.5, 1e-6)
    return np.abs(dx * ddy - dy * ddx) / denom


def precompute_speed_profile(wp, a_brake):
    n  = len(wp)
    ds = np.array([
        np.linalg.norm(wp[(i + 1) % n, :2] - wp[i, :2])
        for i in range(n)
    ])
    v_profile = wp[:, 2].copy()
    for _ in range(3):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v_lim = np.sqrt(v_profile[j] ** 2 + 2.0 * a_brake * ds[i])
            v_profile[i] = min(v_profile[i], v_lim)
    return v_profile


def build_pacejka_lut(pac_d):
    alphas  = np.linspace(-0.30, 0.30, 500)
    Fy_norm = pac_d * np.sin(
        PAC_C * np.arctan(
            PAC_B * alphas
            - PAC_E * (PAC_B * alphas - np.arctan(PAC_B * alphas))
        )
    )
    return alphas, Fy_norm


def pacejka_inv(Fy_req, pac_alpha_lut, pac_Fy_lut):
    clamped = float(np.clip(Fy_req,
                            pac_Fy_lut[0]  * 0.99,
                            pac_Fy_lut[-1] * 0.99))
    return float(np.interp(clamped, pac_Fy_lut, pac_alpha_lut))


def find_lookahead(wp, pos, lookahead, near_idx):
    n = len(wp)
    dists = np.linalg.norm(wp[:, :2] - pos, axis=1)
    candidates = [(near_idx + i) % n for i in range(50)]
    near = candidates[int(np.argmin([dists[i] for i in candidates]))]

    arc = 0.0
    for i in range(n):
        i0  = (near + i)     % n
        i1  = (near + i + 1) % n
        seg = float(np.linalg.norm(wp[i1, :2] - wp[i0, :2]))
        if arc + seg >= lookahead:
            alpha   = float(np.clip((lookahead - arc) / max(seg, 1e-6), 0.0, 1.0))
            goal_xy = (1.0 - alpha) * wp[i0, :2] + alpha * wp[i1, :2]
            return goal_xy, near
        arc += seg
    return None, near


def compute_action(wp, v_profile, pos, yaw, speed, near_idx, steer_prev,
                   params, pac_alpha_lut, pac_Fy_lut):
    v  = max(speed, 0.3)
    L1 = max(params['L1_MIN'], params['L1_K'] * v)

    goal, near = find_lookahead(wp, pos, L1, near_idx)
    if goal is None:
        return 0.0, v_profile[near_idx], near_idx

    dx  = goal[0] - pos[0]
    dy  = goal[1] - pos[1]
    bearing = np.arctan2(dy, dx)
    eta = float(np.arctan2(np.sin(bearing - yaw), np.cos(bearing - yaw)))

    a_lat   = 2.0 * v**2 * np.sin(eta) / L1
    Fzf     = MASS * 9.81 * LR / WHEELBASE
    F_yf    = MASS * a_lat * LR / WHEELBASE
    alpha_f = pacejka_inv(F_yf / (MU * Fzf), pac_alpha_lut, pac_Fy_lut)

    delta_kin = np.arctan(WHEELBASE * 2.0 * np.sin(eta) / max(L1, 1e-3))
    raw_steer = float(np.clip(delta_kin + alpha_f, -MAX_STEER, MAX_STEER))

    alpha = params['STEER_ALPHA']
    steer = alpha * steer_prev + (1.0 - alpha) * raw_steer

    target_speed = float(v_profile[near])
    return steer, target_speed, near


def run_trial(env, wp, spawn, params):
    pac_alpha_lut, pac_Fy_lut = build_pacejka_lut(params['PAC_D'])
    v_profile = precompute_speed_profile(wp, params['A_BRAKE'])

    obs, _, _, _ = env.reset(spawn)
    elapsed   = 0.0
    departed  = False
    steer     = 0.0
    tgt_speed = 1.0
    near_idx  = 0
    step_count = 0

    # 초기 진행 방향 확인
    yaw0 = float(obs['poses_theta'][0])
    fwd  = np.array([np.cos(yaw0), np.sin(yaw0)])
    wp_dir = wp[1, :2] - wp[0, :2]
    if np.dot(fwd, wp_dir) < 0:
        wp = wp[::-1]
        v_profile = v_profile[::-1]

    start_xy = spawn[0, :2]
    max_dist  = 0.0
    best_score = PENALTY

    while elapsed < TIMEOUT:
        pos   = np.array([obs['poses_x'][0], obs['poses_y'][0]])
        yaw   = float(obs['poses_theta'][0])
        speed = float(np.hypot(obs['linear_vels_x'][0], obs['linear_vels_y'][0]))

        if step_count % CTRL_STEPS == 0:
            steer, tgt_speed, near_idx = compute_action(
                wp, v_profile, pos, yaw, speed, near_idx, steer,
                params, pac_alpha_lut, pac_Fy_lut)

        obs, _, _, _ = env.step(np.array([[steer, tgt_speed]]))
        elapsed += 0.01
        step_count += 1

        if obs['collisions'][0]:
            # 충돌: 더 많이 진행할수록 낮은 점수 (좋은 방향)
            dists = np.linalg.norm(wp[:, :2] - pos, axis=1)
            progress = int(np.argmin(dists))
            return PENALTY + (N_WP - progress)  # 10000~10417

        dist_from_start = float(np.linalg.norm(pos - start_xy))
        if dist_from_start > 5.0:
            departed = True
        if departed and dist_from_start < 1.5:
            return elapsed  # 완주: 랩타임(초) 반환 — 가장 낮은 점수

    # 타임아웃 — 완주 못했으므로 진행 거리로 부분 점수
    dists = np.linalg.norm(wp[:, :2] - np.array([obs['poses_x'][0], obs['poses_y'][0]]), axis=1)
    progress = int(np.argmin(dists))
    return PENALTY + (N_WP - progress)


def update_map_controller(best_params):
    with open(MAP_NODE_PATH) as f:
        code = f.read()

    replacements = {
        'L1_MIN':     f"self.L1_MIN = {best_params['L1_MIN']:.4f}",
        'L1_K':       f"self.L1_K   = {best_params['L1_K']:.4f}",
        'STEER_ALPHA': f"self.STEER_ALPHA = {best_params['STEER_ALPHA']:.4f}",
        'A_BRAKE':    f"self.A_BRAKE = {best_params['A_BRAKE']:.4f}",
        'PAC_D':      f"self.PAC_D = {best_params['PAC_D']:.4f}",
    }
    for key, new_line in replacements.items():
        code = re.sub(rf'self\.{key}\s*=\s*[^\n]+', new_line, code)

    with open(MAP_NODE_PATH, 'w') as f:
        f.write(code)
    print('  → map_controller_node.py 업데이트 완료')


def main():
    print('=' * 62)
    print(' MAP 컨트롤러 파라미터 최적화 (Optuna + f1tenth_gym)')
    print('=' * 62)

    wp = load_waypoints(WP_PATH)
    print(f'  웨이포인트: {len(wp)}개 로드')
    print(f'  속도 범위: {wp[:,2].min():.2f}~{wp[:,2].max():.2f} m/s')

    yaw0  = np.arctan2(wp[1, 1] - wp[0, 1], wp[1, 0] - wp[0, 0])
    spawn = np.array([[wp[0, 0], wp[0, 1], yaw0]])

    env = gym.make(
        'f110_gym:f110-v0',
        num_agents=1,
        map=MAP_PATH,
        map_ext='.png',
        integrator=Integrator.RK4,
        timestep=0.01,
    )
    print(f'  시뮬레이터 준비 완료\n')

    baseline = {
        'L1_MIN':     0.45,
        'L1_K':       0.12,
        'STEER_ALPHA': 0.35,
        'A_BRAKE':    9.4675,
        'PAC_D':      1.0,
    }
    print('기준값 평가 중...')
    t0 = run_trial(env, wp, spawn, baseline)
    lapped = t0 < PENALTY
    print(f'  기준: {"완주 " + str(round(t0,2)) + "s" if lapped else "미완주 (score=" + str(round(t0,0)) + ")"}\n')

    best_so_far = [t0]

    def objective(trial):
        params = {
            'L1_MIN':      trial.suggest_float('L1_MIN',     0.25, 1.0),
            'L1_K':        trial.suggest_float('L1_K',       0.08, 0.35),
            'STEER_ALPHA': trial.suggest_float('STEER_ALPHA', 0.0,  0.6),
            'A_BRAKE':     trial.suggest_float('A_BRAKE',    5.0,  15.0),
            'PAC_D':       trial.suggest_float('PAC_D',      0.6,  1.4),
        }
        t = run_trial(env, wp, spawn, params)

        marker = ''
        if t < best_so_far[0]:
            best_so_far[0] = t
            marker = '  ★'
        lapped = t < PENALTY
        status = f'{t:.2f}s 완주' if lapped else f'충돌({int(N_WP - (t - PENALTY))}wp)'
        print(f'  [{trial.number:3d}] {status:12s}  '
              f'L1={params["L1_MIN"]:.2f}+{params["L1_K"]:.3f}v  '
              f'α={params["STEER_ALPHA"]:.2f}  '
              f'BRK={params["A_BRAKE"]:.1f}  '
              f'D={params["PAC_D"]:.2f}{marker}')
        return t

    sampler = optuna.samplers.TPESampler(seed=42)
    study   = optuna.create_study(direction='minimize', sampler=sampler)
    study.enqueue_trial(baseline)

    print('최적화 시작 (10분 제한, 최대 300 trials)...')
    print(f'  {"#":>3}  {"결과":>12}  {"L1_MIN+K*v":>14}  {"α":>5}  {"BRK":>5}  {"D":>5}')
    print('-' * 62)

    study.optimize(objective, timeout=600, n_trials=300)
    env.close()

    best   = study.best_params
    best_t = study.best_value
    n_done = len(study.trials)

    print('\n' + '=' * 62)
    print(f' 최적화 완료: {n_done}회 시도')
    print('=' * 62)
    if t0 < PENALTY:
        print(f'  기준 랩타임  : {t0:.3f}s')
    else:
        print(f'  기준        : 미완주')
    if best_t < PENALTY:
        print(f'  최적 랩타임 : {best_t:.3f}s')
    else:
        progress = int(N_WP - (best_t - PENALTY))
        print(f'  최적        : 미완주 (최대 {progress}개 웨이포인트 통과)')
    print('\n  최적 파라미터:')
    for k, v in best.items():
        old = baseline[k]
        print(f'    {k:12s}: {old} → {v:.4f}')

    print('\n  map_controller_node.py에 최적값 반영 중...')
    update_map_controller(best)
    print('  완료. ros2 run 으로 재시작하면 적용됩니다.')


if __name__ == '__main__':
    main()
