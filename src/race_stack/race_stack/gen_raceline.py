#!/usr/bin/env python3
"""
gen_raceline.py — TUMFTM 방식 최소 곡률 레이싱라인 생성기

알고리즘: Heilmeier et al. "Minimum Curvature Trajectory Planning and Control
for an Autonomous Racecar" (2019), DOI: 10.1080/00423114.2019.1631455

의존 패키지: numpy, scipy, osqp, opencv-python, Pillow
  pip install numpy scipy osqp opencv-python Pillow

사용법:
  python3 gen_raceline.py                          # 대화형 맵 선택
  python3 gen_raceline.py --map sohot_clean        # 맵 지정
  python3 gen_raceline.py --map sohot_clean --vmax 8.0 --vmin 1.0
"""

import os, sys, csv, argparse, math
import numpy as np
import cv2
from PIL import Image
from scipy.interpolate import CubicSpline, interp1d
from scipy.spatial import cKDTree
import osqp
import scipy.sparse as sp

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPS_DIR = os.path.join(_PKG_DIR, 'maps')
WP_DIR   = os.path.join(_PKG_DIR, 'waypoints')

# ══════════════════════════════════════════════════════════════════════════════
# 1. 맵 → 트랙 경계 추출
# ══════════════════════════════════════════════════════════════════════════════
def extract_reftrack(map_name: str, safety_m: float, n_cl: int = 300) -> np.ndarray:
    """
    pgm 맵에서 centerline + 트랙 폭을 추출한다.
    반환: [N, 4] = [x, y, w_right, w_left] (미터)
    """
    yaml_path = os.path.join(MAPS_DIR, map_name + '.yaml')
    img_path  = None
    for ext in ['.pgm', '.png']:
        p = os.path.join(MAPS_DIR, map_name + ext)
        if os.path.exists(p):
            img_path = p; break
    if img_path is None:
        sys.exit(f'맵 이미지 없음: {MAPS_DIR}/{map_name}.pgm')

    import yaml
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    res    = float(cfg['resolution'])
    origin = np.array(cfg['origin'][:2], dtype=float)

    img = np.array(Image.open(img_path).convert('L'))
    h, w = img.shape

    def px2world(px, py):
        x = np.asarray(px, float) * res + origin[0]
        y = (h - np.asarray(py, float)) * res + origin[1]
        return x, y

    free = (img > 128).astype(np.uint8) * 255
    n_lbl, labels, stats, _ = cv2.connectedComponentsWithStats(free)
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    track_mask = (labels == largest).astype(np.uint8) * 255

    contours, hierarchy = cv2.findContours(
        track_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    outer_c = inner_c = None
    for c, hi in zip(contours, hierarchy[0]):
        area = cv2.contourArea(c)
        if hi[3] < 0:
            if outer_c is None or area > cv2.contourArea(outer_c): outer_c = c
        elif area > 500:
            if inner_c is None or area > cv2.contourArea(inner_c): inner_c = c

    if outer_c is None or inner_c is None:
        sys.exit('경계 추출 실패 — 맵을 clean_map.py로 정제하세요')

    def contour_world(c):
        pts = c.reshape(-1, 2)
        x, y = px2world(pts[:, 0], pts[:, 1])
        return np.column_stack([x, y])

    outer_w = contour_world(outer_c)
    inner_w = contour_world(inner_c)

    # 균일 리샘플
    def resample_closed(pts, n):
        closed = np.vstack([pts, pts[0]])
        ds = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        s  = np.concatenate([[0], np.cumsum(ds)])
        s_new = np.linspace(0, s[-1], n, endpoint=False)
        xi = np.interp(s_new, s, closed[:, 0])
        yi = np.interp(s_new, s, closed[:, 1])
        return np.column_stack([xi, yi])

    outer_rs = resample_closed(outer_w, n_cl)

    # 중심선: outer 각 점 → 가장 가까운 inner 점의 중점
    tree_inner = cKDTree(inner_w)
    tree_outer = cKDTree(outer_w)
    _, idx_i = tree_inner.query(outer_rs)
    midpoints = (outer_rs + inner_w[idx_i]) / 2.0

    # 방향 보정 (CCW)
    cx_, cy_ = midpoints[:, 0], midpoints[:, 1]
    area = 0.5 * np.sum(cx_ * np.roll(cy_, -1) - np.roll(cx_, -1) * cy_)
    if area < 0:
        midpoints = midpoints[::-1]

    # 중심선 스플라인 스무딩
    from scipy.interpolate import splprep, splev
    tck, _ = splprep([midpoints[:, 0], midpoints[:, 1]],
                     s=len(midpoints) * 0.01, per=True, k=3)
    u = np.linspace(0, 1, n_cl, endpoint=False)
    cx_s, cy_s = splev(u, tck)
    cl = np.column_stack([cx_s, cy_s])

    # 트랙 폭 계산 (중심선 → outer/inner 거리)
    d_o, _ = tree_outer.query(cl)
    d_i, _ = tree_inner.query(cl)

    # 안전 마진 적용 후 최소 폭 보장
    min_w = 0.20  # 물리 최솟값
    w_right = np.maximum(d_o - safety_m, min_w)
    w_left  = np.maximum(d_i - safety_m, min_w)

    reftrack = np.column_stack([cl, w_right, w_left])
    print(f'  트랙 추출: {n_cl}pts  길이≈{np.sum(np.linalg.norm(np.diff(cl, axis=0), axis=1)):.1f}m')
    print(f'  폭: right {d_o.min():.2f}~{d_o.max():.2f}m  left {d_i.min():.2f}~{d_i.max():.2f}m')
    return reftrack


# ══════════════════════════════════════════════════════════════════════════════
# 2. 균일 호 길이 리샘플
# ══════════════════════════════════════════════════════════════════════════════
def interp_track(reftrack: np.ndarray, stepsize: float) -> np.ndarray:
    """호 길이 기준으로 reftrack을 균일 간격 리샘플."""
    pts = reftrack[:, :2]
    closed = np.vstack([pts, pts[0]])
    ds = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s  = np.concatenate([[0], np.cumsum(ds)])
    n_new = max(3, int(s[-1] / stepsize))
    s_new = np.linspace(0, s[-1], n_new, endpoint=False)

    out = np.zeros((n_new, 4))
    for col in range(4):
        vals_cl = np.append(reftrack[:, col], reftrack[0, col])
        out[:, col] = np.interp(s_new, s, vals_cl)
    return out


# ══════════════════════════════════════════════════════════════════════════════
# 3. 3차 스플라인 시스템 (TUMFTM calc_splines 재구현)
# ══════════════════════════════════════════════════════════════════════════════
def calc_splines(path: np.ndarray) -> tuple:
    """
    닫힌 경로에 곡률 연속 3차 스플라인 피팅.
    path: [N+1, 2] — 마지막 행이 첫 행과 동일한 닫힌 경로

    반환:
      coeffs_x, coeffs_y: [N, 4] 스플라인 계수 [a0, a1, a2, a3]
      M: [4N, 4N] 선형 시스템 행렬
      normvec: [N, 2] 정규화된 법선벡터 (오른쪽 방향)
    """
    assert np.allclose(path[0], path[-1]), "닫힌 경로여야 합니다 (첫점 = 마지막점)"
    N = path.shape[0] - 1  # 스플라인 구간 수 = 제어점 수

    # 스케일링: 인접 구간 거리 비
    el = np.linalg.norm(np.diff(path, axis=0), axis=1)  # [N]
    el_ext = np.append(el, el[0])
    scaling = el_ext[:-1] / el_ext[1:]  # [N]

    # 4N × 4N 선형 시스템
    M  = np.zeros((4 * N, 4 * N))
    bx = np.zeros(4 * N)
    by = np.zeros(4 * N)

    # 각 스플라인 구간 i (0 ~ N-2): 위치 연속 + C1/C2 연속
    for i in range(N - 1):
        j = i * 4
        # P_i(0) = p_i
        M[j, j] = 1.0
        bx[j] = path[i, 0]; by[j] = path[i, 1]
        # P_i(1) = p_{i+1}
        M[j+1, j:j+4] = [1, 1, 1, 1]
        bx[j+1] = path[i+1, 0]; by[j+1] = path[i+1, 1]
        # C1: P'_i(1) = s * P'_{i+1}(0)
        M[j+2, j+1] = 1; M[j+2, j+2] = 2; M[j+2, j+3] = 3
        M[j+2, j+5] = -scaling[i]
        # C2: P''_i(1) = s^2 * P''_{i+1}(0)
        M[j+3, j+2] = 2; M[j+3, j+3] = 6
        M[j+3, j+6] = -2 * scaling[i]**2

    # 마지막 구간 N-1: 위치만 + 닫힘 조건
    j = (N - 1) * 4
    M[j,   j:j+4] = [1, 0, 0, 0]
    bx[j] = path[N-1, 0]; by[j] = path[N-1, 1]
    M[j+1, j:j+4] = [1, 1, 1, 1]
    bx[j+1] = path[0, 0]; by[j+1] = path[0, 1]
    # 닫힘 C1
    M[j+2, j+1] = scaling[N-1]
    M[j+2, j+2] = 2 * scaling[N-1]
    M[j+2, j+3] = 3 * scaling[N-1]
    M[j+2, 1] = -1
    # 닫힘 C2
    M[j+3, j+2] = 2 * scaling[N-1]**2
    M[j+3, j+3] = 6 * scaling[N-1]**2
    M[j+3, 2] = -2

    sol_x = np.linalg.solve(M, bx)
    sol_y = np.linalg.solve(M, by)

    coeffs_x = sol_x.reshape(N, 4)  # [a0, a1, a2, a3] per segment
    coeffs_y = sol_y.reshape(N, 4)

    # 법선벡터: 헤딩(a1)의 수직 방향 (오른쪽)
    # tangent = [a1_x, a1_y], normal = [a1_y, -a1_x] (right-hand)
    normvec = np.stack([coeffs_y[:, 1], -coeffs_x[:, 1]], axis=1)
    norms   = np.linalg.norm(normvec, axis=1, keepdims=True)
    norms   = np.maximum(norms, 1e-9)
    normvec /= norms

    return coeffs_x, coeffs_y, M, normvec


# ══════════════════════════════════════════════════════════════════════════════
# 4. 최소 곡률 QP (TUMFTM opt_min_curv 재구현)
# ══════════════════════════════════════════════════════════════════════════════
def opt_min_curv(reftrack: np.ndarray,
                 normvec:  np.ndarray,
                 M_spl:   np.ndarray,
                 w_veh:   float) -> np.ndarray:
    """
    최소 곡률 경로 최적화 (OSQP QP 풀이).

    수식 (Heilmeier et al. 2019):
      경로: p_i = p_ref_i + α_i * n_i
      목적: min Σ κ_i²(α)
      → QP: min ½ α' H α + f' α
      제약: -w_left[i]+W/2 ≤ α[i] ≤ w_right[i]-W/2

    H = E'E,  f = 2 E' κ_ref
    E = diag(cx) T_ny − diag(cy) T_nx
    cx_i = x'_i / ||(x',y')||^(3/2),  cy_i = y'_i / ||(x',y')||^(3/2)
    T_nx = (A_ex_c @ A_inv) @ M_x_norm
    T_ny = (A_ex_c @ A_inv) @ M_y_norm
    """
    N = reftrack.shape[0]

    # ── 추출 행렬 ──
    # A_ex_c: 각 스플라인 a2 계수(= x''/2) 추출 → x''_i = 2*a2_i
    A_ex_c = np.zeros((N, 4 * N))
    for i in range(N):
        A_ex_c[i, i * 4 + 2] = 2.0  # 2*a2_i

    # A_ex_b: a1 계수(= x'_i at t=0) 추출
    A_ex_b = np.zeros((N, 4 * N))
    for i in range(N):
        A_ex_b[i, i * 4 + 1] = 1.0

    A_inv = np.linalg.inv(M_spl)
    T_c   = A_ex_c @ A_inv   # [N, 4N]: maps q → x''

    # ── 법선벡터를 RHS에 투영하는 행렬 M_x, M_y ──
    # alpha[i]가 q_x[j=4i]와 q_x[j=4i+1]을 normvec[i,0]만큼 이동
    M_x = np.zeros((4 * N, N))
    M_y = np.zeros((4 * N, N))
    for i in range(N):
        j = i * 4
        i_next = (i + 1) % N
        M_x[j,      i]      = normvec[i,      0]
        M_x[j + 1,  i_next] = normvec[i_next, 0]
        M_y[j,      i]      = normvec[i,      1]
        M_y[j + 1,  i_next] = normvec[i_next, 1]

    # ── 참조선 좌표를 스플라인 RHS 벡터 [4N]로 구성 ──
    # 스플라인 시스템의 position 행(4i, 4i+1)에 좌표 배치
    q_x = np.zeros(4 * N)
    q_y = np.zeros(4 * N)
    for i in range(N):
        j = i * 4
        i_next = (i + 1) % N
        q_x[j]   = reftrack[i,      0]
        q_x[j+1] = reftrack[i_next, 0]
        q_y[j]   = reftrack[i,      1]
        q_y[j+1] = reftrack[i_next, 1]

    # 스플라인 RHS → x'_i = a1_i at t=0
    x_prime = (A_ex_b @ A_inv @ q_x)  # [N]
    y_prime = (A_ex_b @ A_inv @ q_y)  # [N]

    # ── 곡률 분모 (= ||(x',y')||^(3/2)) ──
    denom = np.maximum((x_prime**2 + y_prime**2) ** 1.5, 1e-9)  # [N]
    cx = x_prime / denom   # [N]
    cy = y_prime / denom   # [N]

    # ── T_nx, T_ny: α → Δx''(제2도함수 변화량) ──
    # T_c: [N, 4N],  M_x: [4N, N]  → T_nx: [N, N]
    T_nx = T_c @ M_x   # [N, N]
    T_ny = T_c @ M_y   # [N, N]

    # ── E 행렬: α → Δκ ──
    # Δκ_i = cx_i * (T_ny @ α)_i - cy_i * (T_nx @ α)_i
    E = np.diag(cx) @ T_ny - np.diag(cy) @ T_nx   # [N, N]

    # ── 참조선 곡률 ──
    x2_ref = T_c @ q_x   # x''_ref [N] (q_x는 [4N] 벡터)
    y2_ref = T_c @ q_y   # y''_ref [N]
    kappa_ref = cx * y2_ref - cy * x2_ref   # [N]

    # ── QP 행렬 ──
    H = E.T @ E                        # [N, N] PSD
    H = (H + H.T) / 2.0               # 수치 대칭 보장
    f = 2.0 * E.T @ kappa_ref          # [N]

    # ── 제약: -w_left + W/2 ≤ α ≤ w_right - W/2 ──
    dev_right = reftrack[:, 2] - w_veh / 2   # [N]
    dev_left  = reftrack[:, 3] - w_veh / 2   # [N]
    dev_right = np.maximum(dev_right, 0.01)
    dev_left  = np.maximum(dev_left,  0.01)

    # OSQP: min ½ x'Px + q'x  s.t. l ≤ Ax ≤ u
    # 제약을 등가로: -dev_left ≤ α ≤ dev_right → A=I, l=-dev_left, u=dev_right
    prob = osqp.OSQP()
    prob.setup(
        P=sp.csc_matrix(H),
        q=f,
        A=sp.eye(N, format='csc'),
        l=-dev_left,
        u=dev_right,
        verbose=False,
        eps_abs=1e-9,
        eps_rel=1e-9,
        max_iter=50000,
        polish=True,
        polish_refine_iter=10,
    )
    result = prob.solve()
    if result.info.status not in ('solved', 'solved_inaccurate'):
        print(f'  경고: OSQP 상태 = {result.info.status}')

    alpha = np.clip(result.x, -dev_left, dev_right)

    # 곡률 개선 확인
    kappa_new = kappa_ref + E @ alpha
    print(f'  참조선 |κ|_max={np.abs(kappa_ref).max():.4f}  '
          f'레이싱라인 |κ|_max={np.abs(kappa_new).max():.4f} 1/m')
    return alpha


# ══════════════════════════════════════════════════════════════════════════════
# 5. 레이싱라인 생성 + 균일 리샘플
# ══════════════════════════════════════════════════════════════════════════════
def build_raceline(reftrack: np.ndarray,
                   normvec:  np.ndarray,
                   alpha:    np.ndarray,
                   stepsize: float = 0.1) -> tuple:
    """
    α 오프셋을 적용한 레이싱라인을 생성하고 CubicSpline으로 균일 리샘플.
    반환: (raceline [M,2], el_lengths [M], kappa [M])
    """
    N = len(reftrack)
    nodes = reftrack[:, :2] + alpha[:, None] * normvec   # [N, 2]

    # 노드 간 호 길이
    ds_n  = np.linalg.norm(np.diff(np.vstack([nodes, nodes[0]]), axis=0), axis=1)
    total = ds_n.sum()

    # splprep (periodic, 가벼운 스무딩) → 진동 방지
    from scipy.interpolate import splprep, splev
    tck, _ = splprep([nodes[:, 0], nodes[:, 1]],
                     s=len(nodes) * 0.002, per=True, k=3)

    M   = max(3, int(total / stepsize))
    u_u = np.linspace(0, 1, M, endpoint=False)

    xi, yi = splev(u_u, tck)
    rl = np.column_stack([xi, yi])

    # 각 점 간 거리
    el = np.linalg.norm(np.diff(np.vstack([rl, rl[0]]), axis=0), axis=1)

    # 1·2차 도함수로 곡률 계산
    dx, dy   = splev(u_u, tck, der=1)
    ddx, ddy = splev(u_u, tck, der=2)
    kappa = np.abs(dx * ddy - dy * ddx) / np.maximum((dx**2 + dy**2) ** 1.5, 1e-9)

    print(f'  레이싱라인: {M}pts  길이={total:.1f}m  |κ|_max={kappa.max():.4f} 1/m')
    return rl, el, kappa


# ══════════════════════════════════════════════════════════════════════════════
# 6. 속도 프로파일 (전방/역방향 스윕)
# ══════════════════════════════════════════════════════════════════════════════
def calc_vel_profile(kappa:    np.ndarray,
                     el:       np.ndarray,
                     mu:       float,
                     v_max:    float,
                     v_min:    float,
                     a_lon_acc: float,
                     a_lon_brk: float) -> np.ndarray:
    """
    횡방향 마찰 한계 + 전/후방 종방향 가속도로 속도 프로파일 계산.
    """
    n = len(kappa)
    g = 9.81
    a_lat_max = mu * g

    # 횡가속도 한계: v = sqrt(a_lat_max / |κ|)
    v_lat = np.where(kappa > 1e-4,
                     np.sqrt(a_lat_max / kappa),
                     v_max)
    v = np.minimum(np.full(n, v_max), v_lat)

    # 역방향 스윕 (제동 한계)
    for _ in range(3):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v_lim = math.sqrt(v[j]**2 + 2.0 * a_lon_brk * el[i])
            if v[i] > v_lim:
                v[i] = v_lim

    # 전방향 스윕 (가속 한계)
    for _ in range(3):
        for i in range(n):
            j = (i + 1) % n
            v_lim = math.sqrt(v[i]**2 + 2.0 * a_lon_acc * el[i])
            if v[j] > v_lim:
                v[j] = v_lim

    v = np.clip(v, v_min, v_max)
    lap_t = float(np.sum(el / np.maximum(v, 0.01)))
    print(f'  속도: {v.min():.2f}~{v.max():.2f} m/s  예상 랩타임={lap_t:.1f}s')
    above = (v > 7.319).mean() * 100
    if above > 0:
        print(f'  동역학 모델(v>7.319m/s) 구간: {above:.1f}%  → 슬립 발생 가능')
    return v


# ══════════════════════════════════════════════════════════════════════════════
# 7. 오버레이 PNG 저장
# ══════════════════════════════════════════════════════════════════════════════
def save_overlay(map_name, raceline, v, wp_path):
    try:
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.colors import Normalize
        from matplotlib.collections import LineCollection

        yaml_path = os.path.join(MAPS_DIR, map_name + '.yaml')
        import yaml
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)
        res    = float(cfg['resolution'])
        origin = np.array(cfg['origin'][:2])

        img_path = None
        for ext in ['.pgm', '.png']:
            p = os.path.join(MAPS_DIR, map_name + ext)
            if os.path.exists(p):
                img_path = p; break
        img = np.array(Image.open(img_path).convert('L'))
        h_img = img.shape[0]

        px = ((raceline[:, 0] - origin[0]) / res).astype(int)
        py = (h_img - (raceline[:, 1] - origin[1]) / res).astype(int)
        px = np.append(px, px[0]); py = np.append(py, py[0])
        v_ext = np.append(v, v[0])

        fig = Figure(figsize=(9, 9))
        FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)
        ax.imshow(img, cmap='gray', origin='upper')
        ax.set_aspect('equal'); ax.axis('off')

        pts  = np.array([px, py]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc   = LineCollection(segs, cmap='RdYlGn',
                              norm=Normalize(v.min(), v.max()),
                              linewidth=2.5, zorder=5)
        lc.set_array(v_ext)
        ax.add_collection(lc)
        ax.plot(px[0], py[0], 'b*', ms=15, zorder=6)
        fig.colorbar(lc, ax=ax, label='speed (m/s)', fraction=0.03, pad=0.02)
        ax.set_title(f'{map_name}  —  {len(raceline)}pts '
                     f'({v.min():.1f}~{v.max():.1f} m/s)', fontsize=10)

        out = os.path.splitext(wp_path)[0] + '.png'
        fig.tight_layout()
        fig.savefig(out, dpi=130, bbox_inches='tight')
        print(f'  오버레이 PNG: {out}')
    except Exception as e:
        print(f'  오버레이 저장 실패: {e}')


# ══════════════════════════════════════════════════════════════════════════════
# 8. main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    p = argparse.ArgumentParser(description='TUMFTM 최소 곡률 레이싱라인 생성기')
    p.add_argument('--map',    default=None,  help='맵 이름 (생략 시 대화형 선택)')
    p.add_argument('--vmax',   type=float, default=8.0,  help='최대 속도 (m/s)')
    p.add_argument('--vmin',   type=float, default=1.0,  help='최소 속도 (m/s)')
    # 2026-08-09 실측 추정 0.40 (구 1.0489). 과대평가하면 레이싱라인 속도가
# 실제 그립을 넘어 코너에서 밀려난다 — 안전 방향으로 낮춘다.
    p.add_argument('--mu',     type=float, default=0.40, help='마찰 계수')
    p.add_argument('--wveh',   type=float, default=0.31,  help='차량 폭 (m)')
    p.add_argument('--safety', type=float, default=0.15,  help='벽 안전 마진 (m)')
    p.add_argument('--acc',    type=float, default=5.0,  help='가속도 (m/s²)')
    p.add_argument('--brk',    type=float, default=8.0,  help='제동 (m/s²)')
    p.add_argument('--step',   type=float, default=0.1,  help='웨이포인트 간격 (m)')
    args = p.parse_args()

    # 맵 선택
    if args.map:
        map_name = args.map
    else:
        maps = sorted(set(
            f.rsplit('.', 1)[0]
            for f in os.listdir(MAPS_DIR)
            if f.endswith(('.pgm', '.png'))
        ))
        print('\n사용 가능한 맵:')
        for i, m in enumerate(maps, 1):
            print(f'  {i}. {m}')
        val = input('맵 이름 또는 번호: ').strip()
        map_name = maps[int(val) - 1] if val.isdigit() else val

    wp_path = os.path.join(WP_DIR, f'{map_name}_wp.csv')
    os.makedirs(WP_DIR, exist_ok=True)

    print(f'\n맵: {map_name}  vmax={args.vmax}  wveh={args.wveh}  safety={args.safety}')

    # ── 단계별 실행 ──
    print('\n[1] 트랙 경계 추출...')
    reftrack = extract_reftrack(map_name, safety_m=args.safety)

    print('\n[2] 균일 리샘플 (0.5m 간격)...')
    reftrack_u = interp_track(reftrack, stepsize=0.5)
    N = len(reftrack_u)
    print(f'  {N}pts')

    print('\n[3] 3차 스플라인 피팅...')
    path_cl = np.vstack([reftrack_u[:, :2], reftrack_u[0, :2]])
    coeffs_x, coeffs_y, M_spl, normvec = calc_splines(path_cl)

    print('\n[4] 최소 곡률 QP (OSQP)...')
    alpha = opt_min_curv(reftrack_u, normvec, M_spl, w_veh=args.wveh)
    print(f'  α: [{alpha.min():.3f}, {alpha.max():.3f}] m')

    print('\n[5] 레이싱라인 생성...')
    raceline, el, kappa = build_raceline(reftrack_u, normvec, alpha,
                                         stepsize=args.step)

    print('\n[6] 속도 프로파일...')
    v = calc_vel_profile(kappa, el,
                         mu=args.mu, v_max=args.vmax, v_min=args.vmin,
                         a_lon_acc=args.acc, a_lon_brk=args.brk)

    # ── 저장 ──
    print('\n[7] 저장...')
    with open(wp_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for (x, y), vi in zip(raceline, v):
            w.writerow([f'{x:.4f}', f'{y:.4f}', f'{vi:.4f}'])
    print(f'  웨이포인트: {len(raceline)}pts → {wp_path}')

    save_overlay(map_name, raceline, v, wp_path)

    print('\n완료! 다음 단계:')
    print(f'  race_script.yaml: map_name: {map_name}')
    print(f'  SPEED_SCALE 1.0 으로 시작 (웨이포인트가 이미 고속 최적화됨)')


if __name__ == '__main__':
    main()
