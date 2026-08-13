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
    # ── 소비자가 안 정해도 동작하도록 하는 기본값 ─────────────────────
    #   frenet_planner 와 stanley_controller 가 각자 파라미터로 덮어쓴다.
    #   여기 기본값이 없으면 한쪽만 갱신했을 때 AttributeError 로 죽는다.
    fr_min_room = 0.22      # 통과 여유 하한 [m] = 차 반폭 0.17 + 여유 0.05
    fr_gap_hyst = 0.10      # 반대쪽 통로로 갈아탈 때 요구하는 우세폭 [m]

    def _wall_room(self, flat: np.ndarray) -> np.ndarray:
        """각 점에서 맵 벽까지의 거리 [m]. 맵이 없으면 +inf.

        거리변환을 기동 시 한 번 만들어 두므로 조회는 배열 인덱싱 한 번이다.
        맵 밖으로 나간 점은 0 (= 갈 수 없는 곳) 으로 본다.
        """
        wd = getattr(self, 'wall_dist_m', None)
        if wd is None or self.map_meta is None:
            return np.full(len(flat), np.inf)
        h, w = self.map_meta[0], self.map_meta[1]
        res, ox, oy = self.map_meta[2], self.map_meta[3], self.map_meta[4]
        img = _world_to_img(flat, h, res, ox, oy)
        ix, iy = img[:, 0], img[:, 1]
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        out = np.zeros(len(flat))
        out[ok] = wd[iy[ok], ix[ok]]
        return out

    @staticmethod
    def _runs(mask: np.ndarray):
        """연속된 True 구간을 [(시작, 끝+1), ...] 로. 빈 배열이면 []."""
        out = []
        i = 0
        n = len(mask)
        while i < n:
            if mask[i]:
                j = i
                while j + 1 < n and mask[j + 1]:
                    j += 1
                out.append((i, j + 1))
                i = j + 1
            else:
                i += 1
        return out

    def _frenet_select(self, px: float, py: float, yaw: float) -> float:
        """후보 오프셋 중 하나를 고른다 (m, 좌측 +).

        ★★ 2026-08-11 재작성. 그 전 판정이 **레이싱라인 자신을 27% 의 프레임에서
            "트랙 밖" 으로 탈락시키고 있었다.** ★★

          옛 방식은 `inside = 마스크안(모든 점).all()` 이었다. 호라이즌이
          2.9~5.0m 인데 그 구간 전체가 벽에서 track_margin(0.28m) 밖이어야
          통과다. cool_clean 은 라인 점의 0.7% 가 0.28m 보다 벽에 가까운데,
          5m 창에 `all()` 을 걸면 그 0.7% 가 **27% 프레임 탈락**으로 증폭된다.
          17개 후보 중 중앙 5개만 살아남고 전부 살아남는 프레임은 0% 였다.

          결과가 로그에 그대로 있었다 — **장애물이 없을 때 더 많이 움직였다**:
              q1(장애물 없음)  |d| 중앙 0.075m  p90 0.375m  |d|>0.1 이 31%
              q2(장애물 있음)  |d| 중앙 0.075m  p90 0.225m           17%
          "회피가 안 된다" 의 절반은 회피 로직이 아니라 이 판정이었다.

        이제 예/아니오 대신 **통과 여유(room)** 를 잰다:

            room[i] = min over 경로점 ( 벽까지 거리 , 장애물까지 거리 )
                    = 그 후보 라인으로 갈 때 차가 확보하는 최소 반경

          · 벽 거리는 거리변환 조회 한 번 (마스크와 같은 비용)
          · 실행가능 = 벽여유 ≥ fr_min_room **그리고** 장애물여유 ≥ fr_clear
            두 임계를 따로 두는 이유는 아래 본문 주석 참고 — 합치면 장애물
            요구치가 조용히 내려간다.
            fr_min_room 0.22m 면 두 트랙 모두 레이싱라인 탈락 0% 다
            (실측 라인→벽 최소: cool_clean 0.280m, fun_clean 0.360m)

        그리고 **"더 넓은 쪽" 을 실제로 고른다** (2026-08-11 요구사항):

          실행가능한 후보들을 연속 구간(run)으로 묶으면 각 run 이 "그쪽으로
          지나갈 수 있는 통로" 다. run 의 점수는 그 안의 **최대 room** —
          곧 그쪽으로 갔을 때 낼 수 있는 가장 넓은 여유다. 점수가 큰 run 을
          고른다. 장애물이 왼쪽에 치우쳐 있으면 오른쪽 run 의 room 이 크므로
          자동으로 오른쪽을 고른다.

          ★ 좌우 뒤집힘 방지: 지금 쓰는 오프셋이 든 run 을 계속 쓴다. 다른
            run 이 fr_gap_hyst 이상 더 넓을 때만 갈아탄다. 이게 없으면
            장애물 정면에서 좌우가 프레임마다 바뀌어 조향이 진동한다.

        고른 run 안에서는 예전 비용으로 한 점을 정한다 — 라인 복귀력(w_offset)이
        살아 있으므로 **장애물이 지나가면 저절로 d=0 으로 돌아온다.**

            비용 = w_offset·|d| + w_change·|d-현재| - w_clear·min(room, cap)

        ★ 비용 벡터를 투표 창만큼 이동평균한다. 라이다 한 프레임의 노이즈로
          목표 오프셋이 튀면 그게 곧 조향 킥이 된다.
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
        else:
            d_obs = np.full(N * M, np.inf)

        # ── 벽까지의 여유 ─────────────────────────────────────────────
        d_wall = (self._wall_room(flat) if self.fr_bounds
                  else np.full(N * M, np.inf))

        # ── 통과 여유 ─────────────────────────────────────────────────
        #   ★ 벽과 장애물은 **요구치가 다르다.** 둘을 min 으로 합쳐 임계 하나를
        #     걸면 안 된다 (2026-08-11 첫 시도에서 실제로 그랬다):
        #       · 벽    — 맵에 있고 안 움직인다. 차 반폭 + 여유(0.22m)면 지나간다
        #       · 장애물 — 위치가 덜 정확하고 움직일 수 있다. 더 크게(0.30m) 잡는다
        #     합치면 장애물 요구치가 0.30 → 0.22 로 조용히 내려가, 검증에서
        #     q1 은 좋아졌는데 **q2 의 회피량까지 같이 줄었다** (|d|>0.1 이 16%→5%).
        #     "여유가 늘었다" 가 아니라 "덜 피하게 됐다" 는 뜻이라 정반대다.
        obs_room  = d_obs.reshape(N, M).min(axis=1)
        wall_room = d_wall.reshape(N, M).min(axis=1)
        # 넓이 순위를 매길 때는 실제로 차가 쓸 수 있는 폭이므로 둘 중 좁은 쪽
        room = np.minimum(obs_room, wall_room)

        d = self.fr_grid
        feasible = (wall_room >= self.fr_min_room) & (obs_room >= self.fr_clear)

        cost = (self.fr_w_off * np.abs(d)
                + self.fr_w_chg * np.abs(d - self._shift_cur)
                - self.fr_w_clr * np.minimum(room, self.fr_clr_cap))
        self._fr_costs.append(cost)
        cost_avg = np.mean(np.stack(self._fr_costs), axis=0)

        runs = self._runs(feasible)
        if runs:
            # ── 어느 쪽이 더 넓은가 ────────────────────────────────────
            scores = [float(room[a:b].max()) for a, b in runs]
            pick = int(np.argmax(scores))
            # 지금 쓰는 오프셋이 든 run 은 웬만하면 유지한다 (좌우 진동 방지)
            cur_i = int(np.argmin(np.abs(d - self._shift_cur)))
            for k, (a, b) in enumerate(runs):
                if a <= cur_i < b:
                    if scores[pick] < scores[k] + self.fr_gap_hyst:
                        pick = k
                    break
            a, b = runs[pick]
            best = a + int(np.argmin(cost_avg[a:b]))
            self._fr_gap_room = scores[pick]
            self._fr_status = 0.0 if abs(float(d[best])) < 1e-3 else 1.0
        else:
            # 전부 막혔다 — 그나마 가장 넓은 쪽으로. 여기까지 오면 속도를
            # 줄이는 게 정답이지만 그건 AEB/감속의 영역이고, 조향은 최선을 다한다.
            best = int(np.argmax(room))
            self._fr_gap_room = float(room[best])
            self._fr_status = 2.0
            self.get_logger().warn(
                f'[frenet] 통과 가능한 후보 없음 — 최대 여유 {room[best]:.2f}m '
                f'(요구 {self.fr_min_room:.2f}m), d={d[best]:+.2f}m. '
                f'감속해야 하는 상황이다',
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
        # ★ 거리장 자체를 보관한다 (2026-08-11). 마스크는 "여기 들어가도 되나" 라는
        #   예/아니오뿐인데, 회피는 "여기가 **얼마나 넓나**" 를 알아야 한다.
        #   _frenet_select 가 후보별 통과 여유(room)를 이걸로 잰다.
        self.wall_dist_m = dist_m

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
        """장애물로 볼 빔만 골라 map 좌표 점으로 돌려준다.

        검사 두 가지의 **합집합**이다. 하나로는 못 잡는 영역이 서로 다르다.

        (A) 기대보다 뚜렷하게 짧게 끊긴 빔        meas < 기대 − deficit
            벽 **근처**의 장애물을 잡는다. 기대거리는 부풀린 맵에서 오므로
            AMCL 오차에 강하다 (_build_raycast_map 참고).
            약점: 기대거리가 짧은 곳(좁은 통로·코너 정면)에서는 deficit 만큼의
            여유가 안 나와 아무것도 못 잡는다.

        (B) 빔 끝점이 **열린 공간 한가운데** 찍혔다  벽까지 거리 > free_gap
            트랙 한가운데 놓인 장애물을 잡는다. "거리 차이" 가 아니라 "그 점이
            벽에서 얼마나 떨어졌나" 를 보므로 좁은 통로에서도 그대로 동작하고,
            AMCL 오차에도 강하다 (벽 반사는 정의상 벽 근처에 찍힌다).

        ★ 왜 (B) 를 넣었나 — 2026-08-09 q2 bag 6개를 재생해 실측했다.
          가려진 장애물(벽 뒤)을 제외한 진짜 검출률:

              거리        0.3~1m   1~2m   2~3m   3~4m   4~5m
              (A)만         32%    25%    14%    17%     2%
              (A)|(B)       37%    46%    41%    35%    21%

          호라이즌은 2.9~5.0m 로 맞춰 뒀는데 **검출기가 그 거리에서 안 잡고
          있었다.** 감지했을 땐 이미 1m 앞이고, MAP 은 오프셋을 옮기는 데
          2.2~3.7m 를 달려야 한다 — 구조적으로 못 피한다.
          (B) 를 더하면 2~3m 에서 3배, 4~5m 에서 10배가 된다.

          대안으로 raycast_pose_unc 를 0.25→0.12 로 낮추면 비슷한 재현율이
          나오지만 그건 **벽을 장애물로 보게 만드는 방향**이다 (2026-08-05 에
          직선 9.4m 를 42cm 밀려 달린 그 실패). 우리 AMCL 종방향 오차가 실측
          10~20cm 이므로 0.12 는 쓸 수 없다. (B) 는 unc 를 건드리지 않는다.

          free_gap 은 0.30m 가 무릎점이다 (0.25 로 더 낮춰도 이득이 없다).
          트랙 반폭이 0.5~0.6m 이므로 한가운데 장애물은 0.30 을 넘고, AMCL
          오차 0.2m 를 빼도 여유가 남는다.
        """
        deficit = float(self.get_parameter('raycast_deficit').value)
        fov     = math.radians(float(self.get_parameter('raycast_fov_deg').value))
        bstep   = math.radians(float(self.get_parameter('raycast_beam_deg').value))

        n = len(msg.ranges)
        stride = max(1, int(round(bstep / max(msg.angle_increment, 1e-6))))
        idx = np.arange(0, n, stride)
        # ★ 낮은/얇은 물체(콘, 상대 차 옆면)는 각폭이 1° 보다 작다.
        #   옛 기본값 beam_deg=1.0 은 UST-10LX(0.25°) 를 4개에 하나 건너뛰어
        #   2~4m 앞 10cm 콘을 통째로 놓쳤다. 높은 박스·차는 각폭이 커서 살아남고
        #   낮은 콘만 안 보이던 증상의 소프트웨어 쪽 원인. (스캔 평면 11cm 보다
        #   완전히 낮은 물체는 하드웨어로도 안 보인다.)
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

        ahead = valid & (meas < horizon)
        obs = ahead & (meas < exp - deficit)                     # (A)

        free_gap = float(self.get_parameter('raycast_free_gap').value)
        if free_gap > 0.0 and getattr(self, 'wall_dist_m', None) is not None:
            ang = self.yaw + a_car
            ends = np.vstack((lx0 + meas * np.cos(ang),
                              ly0 + meas * np.sin(ang))).T
            obs = obs | (ahead & (self._wall_room(ends) > free_gap))   # (B)

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
    # ── 동적 장애물(상대차) 추적 ────────────────────────────────────────
    #   1대1 에서는 장애물이 **움직인다.** 지금 자리를 피하면, 내가 거기 도달할
    #   때쯤 상대는 이미 다른 곳에 있다. 4㎧ 로 3m 앞의 상대에게 다가가면
    #   도달까지 0.75s 인데, 상대가 3㎧ 로 달리면 그동안 2.25m 를 간다 —
    #   **현재 위치를 피하는 것은 거의 의미가 없다.**
    #
    #   그래서 검출점을 클러스터로 묶어 프레임 간 대응시키고, 속도를 추정해
    #   도달 시각까지 **쓸고 지나갈 자리 전체**를 장애물로 넘긴다.
    #   (칼만 필터를 쓰지 않는 이유: 상대차는 트랙을 따라 달리므로 등속 가정이
    #    0.5~1s 구간에서는 충분히 맞고, 상태를 늘리면 튜닝할 것만 늘어난다.)
    opp_track     = True    # 동적 예측 사용
    opp_eps       = 0.30    # 클러스터 묶는 거리 [m]
    opp_gate      = 0.80    # 프레임 간 대응 게이트 [m]
    opp_alpha     = 0.6     # 속도 저역통과 (0=즉답, 1=안 움직임)
    opp_min_obs   = 3       # 이만큼 연속 관측돼야 속도를 믿는다
    opp_min_speed = 0.5     # 이보다 느리면 정적 장애물로 본다 [m/s]
    opp_pred_max  = 1.2     # 예측 시간 상한 [s]
    opp_lost_sec  = 0.5     # 이만큼 못 보면 트랙 폐기

    def _cluster_pts(self, pts, eps):
        """격자 해싱 클러스터링. 이웃 격자까지 합쳐 경계에서 갈라지지 않게 한다.

        DBSCAN 을 안 쓰는 이유는 40Hz 안에 끝나야 하기 때문이다 — 점이
        수백 개라 O(n log n) 라이브러리 호출보다 이쪽이 빠르고 의존성도 없다.
        """
        key = np.floor(pts / eps).astype(np.int64)
        lab = -np.ones(len(pts), np.int64)
        table = {}
        for i, k in enumerate(map(tuple, key)):
            table.setdefault(k, []).append(i)
        nxt = 0
        for k, idxs in table.items():
            if lab[idxs[0]] >= 0:
                continue
            # 이웃 격자를 너비우선으로 합친다
            stack, comp = [k], []
            seen = {k}
            while stack:
                c = stack.pop()
                if c not in table:
                    continue
                comp.extend(table[c])
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (c[0] + dx, c[1] + dy)
                        if n not in seen and n in table:
                            seen.add(n); stack.append(n)
            for i in comp:
                lab[i] = nxt
            nxt += 1
        return lab, nxt

    def _predict_obstacles(self, pts, msg):
        """검출점 + **도달 시각까지 상대가 쓸고 갈 자리**를 함께 돌려준다.

        정적 장애물(속도 < opp_min_speed)이면 원래 점을 그대로 낸다 — q2 의
        동작이 바뀌지 않는다.
        """
        if not self.opp_track or len(pts) == 0:
            if len(pts) == 0:
                self._opp_tracks = []
            return pts
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            return pts
        lab, n = self._cluster_pts(pts, self.opp_eps)
        cents = [pts[lab == i].mean(axis=0) for i in range(n)]

        old = getattr(self, '_opp_tracks', [])
        old = [tr for tr in old if t - tr['t'] < self.opp_lost_sec]
        used, new = set(), []
        for ci, c in enumerate(cents):
            best, bd = None, self.opp_gate
            for k, tr in enumerate(old):
                if k in used:
                    continue
                dd = float(np.hypot(*(c - tr['p'])))
                if dd < bd:
                    bd, best = dd, k
            if best is None:
                new.append(dict(p=c, v=np.zeros(2), t=t, n=1, ci=ci))
                continue
            tr = old[best]; used.add(best)
            dt = t - tr['t']
            v = tr['v'] if dt <= 1e-3 else (
                self.opp_alpha * tr['v'] + (1.0 - self.opp_alpha) * (c - tr['p']) / dt)
            new.append(dict(p=c, v=v, t=t, n=tr['n'] + 1, ci=ci))
        self._opp_tracks = new

        out = [pts]
        self._opp_moving = 0
        for tr in new:
            sp = float(np.hypot(*tr['v']))
            if tr['n'] < self.opp_min_obs or sp < self.opp_min_speed:
                continue                       # 정적 — 원래 점으로 충분하다
            self._opp_moving += 1
            # 내가 거기 도달하기까지 걸리는 시간. 접근속도가 0 이하면 멀어지는
            # 중이므로 예측할 것이 없다.
            rel = tr['p'] - self.pos
            d = float(np.hypot(*rel))
            closing = max(abs(self.speed) - float(rel @ tr['v']) / max(d, 1e-3), 0.5)
            tau = float(np.clip(d / closing, 0.0, self.opp_pred_max))
            mem = pts[lab == tr['ci']]
            for f in (0.35, 0.7, 1.0):
                out.append(mem + tr['v'] * (tau * f))
        return np.vstack(out)

    def _obs_horizon(self) -> float:
        """속도에 비례하는 전방 감시 거리(m).

        회피에 필요한 거리는 속도에 비례한다. 고정값(구 3m)은 1.8m/s 에서
        1.7초 앞까지 보는 셈이라 아직 한참 남은 장애물에 미리 반응했다.
        저속에서도 최소 obs_horizon_min 은 확보한다 (0 이면 아무것도 못 본다).
        """
        return float(np.clip(self.obs_h_min + self.obs_h_sec * abs(self.speed),
                             self.obs_h_min, self.obs_h_max))

    @property
    def _avoid_active(self) -> bool:
        """Q2 정적 장애물과 race 1v1 상대 모두 같은 검출기를 쓴다."""
        return self.mode in ('q2', 'race')

    def _update_lane_free(self):
        """차단 투표 창이 과반이면 회피 유지 (통로를 넓혀 놓치지 않음).

        ★ frenet_planner 는 예전엔 이 값을 영원히 True 로 뒀다. 회피를
          시작하는 순간 장애물이 좁은 통로 밖으로 나가 감지가 풀렸다.
        """
        ml = self.free_votes.maxlen or len(self.free_votes)
        if not ml:
            self.lane_free = True
            return
        self.lane_free = sum(self.free_votes) * 2 >= ml

    def _scan_cb(self, msg: LaserScan):
        # 벽 회피는 pose 없이도 (차량 좌표계만으로) 동작해야 한다 —
        # AMCL 수렴 전에도 벽에 스치는 건 막아야 하므로 pose 검사보다 먼저 저장.
        self.latest_scan = msg

        if self.pos is None:
            return
        # ★ 2026-08-13 이전에는 `mode != 'q2'` 라 **race 에서 검출이 통째로 꺼져
        #   있었다.** frenet_planner 는 q2·race 둘 다 뜨므로(race.launch.py) 노드는
        #   살아서 d=0 만 계속 발행한다 — 로그로는 완전히 정상으로 보이는,
        #   조용히 아무 일도 안 하는 종류의 버그였다.
        if not self._avoid_active:
            return          # q1/q3 는 빈 트랙. q2·race 만 검출한다.

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
                self._update_lane_free()
                return
            # 마스크를 다시 걸지 않는다 — 이미 "맵으로 설명되지 않는 점" 만 남았다.
            # ★ 상대차는 움직인다 — 지금 자리가 아니라 **내가 도달할 때 있을 자리**를
            #   피해야 한다 (_predict_obstacles).
            pts_world = self._predict_obstacles(pts_world, msg)
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
            self._update_lane_free()
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
            self._update_lane_free()
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
        self._update_lane_free()
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
