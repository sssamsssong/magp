#!/usr/bin/env python3
"""obstacle_avoid — Frenet 장애물 회피 로직 (컨트롤러 공용).

stanley_controller 에서 **원문 그대로 추출**한 것이다 (2026-08-09). 복사가 아니라
이동이라 사본이 늘지 않는다. 로직은 한 글자도 바꾸지 않았다 — Stanley 가 저속에서
잘 돌고 있고, 여기서 동작이 바뀌면 그 검증이 통째로 무효가 되기 때문이다.

무엇을 하나
──────────
레이싱라인을 기준선으로 삼아 **점별 법선 방향으로 d 만큼 민 후보들**을 만들고,
장애물 여유·트랙 이탈·복귀력·변화량으로 비용을 매겨 최적 d 하나를 고른다.
경로를 새로 만드는 게 아니라 **기준선의 횡오프셋 스칼라 d** 를 정하는 방식이라,
경로 추종 컨트롤러면 무엇이든 소비할 수 있다 (Stanley·MAP 모두).

★ 컨트롤러마다 d 에 수렴하는 속도가 다르다 — 이게 이식 시 유일한 함정이다.
  Stanley 는 δ = ψ_e + atan(k·e/v) 로 전방주시 지연이 없어 즉시 반응한다.
  MAP/L1 은 2차계(ζ=0.707, ωn=√2·v/L1)라 오버슈트는 4.3% 로 작지만
  **상승거리가 1.8·L1/√2 ≈ 1.15m (L1=0.9m)** 나 된다.
  그래서 회피 호라이즌은 **2 × L1 이상**이어야 한다. Stanley 기준으로 맞춘
  obs_horizon_min(0.8m)을 MAP 에 그대로 쓰면 다 피하기 전에 장애물에 닿는다.
  → race_script.yaml frenet.per_controller 에서 컨트롤러별로 따로 준다.

쓰는 법
───────
이 믹스인을 상속하고 아래 속성을 채워두면 된다. (Stanley 가 이미 채우는 것들이고,
frenet_planner 노드도 같은 이름으로 채운다.)
"""

import math
import os

import cv2
import numpy as np
import scipy.spatial.distance
import yaml
from PIL import Image

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


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


# ── 회피 로직 (믹스인) ──────────────────────────────────────────────────────


class ObstacleAvoidMixin:
    """
    필수 속성 (사용하는 쪽에서 준비할 것)
    ─────────────────────────────────────
    상태:   pos, yaw, yaw_rate, speed, wps, _wp_ds, mode, latest_scan
    맵:     map_arr, map_meta, track_mask, ray_occ, track_margin,
            use_bounds, inner_bound, outer_bound
    라이다: laser_dx, laser_dy
    호라이즌: obs_h_sec, obs_h_min, obs_h_max, obs_corr_hw, obs_release_scale
    Frenet: fr_grid, fr_clear, fr_clr_cap, fr_dmax, fr_bounds,
            fr_w_off, fr_w_chg, fr_w_clr, _shift_cur, _fr_costs, _fr_best_d
    검출결과: on_track_pts, blocking_pts, free_votes, lane_free
    발행:   fr_pub, obs_pub   (없으면 None)
    """

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
