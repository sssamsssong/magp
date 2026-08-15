# car2 가 main(1호차)에서 가져와야 할 것 — 2026-08-15 기준

브랜치를 차별로 나눈 뒤(08-13) `main` 쪽에 08/13~15 사흘치 개발이 쌓였다.
`car2` 에만 있는 건 4커밋(맵·라인·캘리브레이션)뿐이고, **`main` 에만 있는 게 50여 커밋**이다.

```bash
git fetch origin
git log --oneline origin/car2..origin/main      # 목록 확인
git merge origin/main                            # 또는 rebase
```

로그 커밋(`logs: ...`)이 절반쯤이라 실제 코드 변경은 25개 정도다.

---

## ★★ 최우선 — 오늘 car2 가 실제로 당한 문제를 그쪽이 고쳐놨다

### `83b9e61` fix(waypoints-kappa): 조향 한계의 85%만 써서 라인 R_min 을 차보다 크게 둔다

**car2 가 이 버그에 두 번 걸렸다.** `gen_waypoints_kappa.py` 의 `MAX_STEER` 이
0.36 하드코딩이라 `config/vesc.yaml` 을 안 읽는다. 결과:

```
map_0813_clean_wp_kappa   못 도는 점 24/400   (κ 1.147 vs 한계 0.995)
0815_clean_wp_kappa       못 도는 점 19/400   (κ 1.241 vs 한계 0.995)
```

08-13 에 `--max-steer 0.29` 로 수동 재생성해 24 → 0 으로 잡았지만, 08-15 에
새 맵을 뽑을 때 또 기본값으로 돌아 19개가 생겼다. **이 커밋을 받으면 자동으로 해결된다.**
받은 뒤 `0815_clean_wp_kappa` 를 다시 뽑을 것.

### `453c2a2` feat(savemap): 출발점을 맵 원점으로 자동 저장 — savepose 는 자리 옮길 때만

**init_pose 가 벽 안에 있어 AMCL 이 안 잡히는 문제를 오늘 두 번 겪었다.**
증상이 `초기중 — 남은항목: AMCL pose` 로만 나타나 원인이 안 보인다
(AMCL 은 `recovery_alpha=0` 이라 init 주변에만 파티클을 뿌린다).
맵을 딸 때 출발점을 원점으로 박아두면 이 실패 자체가 사라진다.

### `137001a` fix(speed): 제동거리만큼 미리 감속하고, 조향 포화면 그 곡률로 속도를 자른다

car2 로그에서 **코너 조향 포화가 32%** 로 측정됐다. 포화 중에는 지령대로 안 도는데
속도 프로파일은 그걸 모른다. 정확히 그 구멍을 메우는 변경이다.

### `71fe9ef` fix(q2): 회피를 완료할 수 없는 속도로 달리고 있었다 — 명시적 속도 상한

car2 에서 같은 계산을 했었다: 필요 거리 ≈ `1.273·L1 + (d_max/shift_rate)·v ≈ 1.28·v`
인데 3.9㎧ 에서 호라이즌 여유가 0.01m 였다. 그쪽은 속도 쪽에서 막았다.

### `418f0e2` fix(1v1): frenet_planner 가 기동에서 즉사하고 있었다 (Marker 미import)

**크래시 버그.** frenet 관련 코드를 받을 거면 이건 반드시 같이 받아야 한다.

---

## ★ 다음 — 라인·맵 도구 전반이 좋아졌다

| 커밋 | 내용 |
|---|---|
| `65567bf` | `./race.sh line` — gen_raceline 을 스크립트로 붙였다 |
| `f1c89bd` | 중심선이 벽에 붙어 라인이 안 나오던 것 — 거리변환 기준으로 |
| `8fc8234` | `--apex-extra` 코너 안쪽에만 여유 (기본 0.40m) |
| `5171cf0` | `./race.sh clean` — 맵 정제를 대화형으로 |
| `3040478` | 맵/웨이포인트 GUI 안내를 **영어로** — 젯슨 matplotlib 에 한글 폰트가 없어 제목이 깨진다 |

마지막 것은 car2 에서도 그대로 겪었다 (`Glyph ... missing from current font` 경고 도배).

## ★ 회피·속도

| 커밋 | 내용 |
|---|---|
| `df19801` | 조향 한계를 넘는 안쪽 오프셋은 버린다 |
| `cae0924` | 후보 선택에 곡률 v_max 를 넣고 TTC 는 cost 에만 |
| `7ee6799` | `a_lon_acc` 를 다시 자동으로 — 1.6 은 μ=0.348 시절 값이었다 |
| `6778e8b` | `speed_scale` 분모가 CSV 열이라 항상 절반으로 깎였다 |

## 도구·운영

| 커밋 | 내용 |
|---|---|
| `34965f3` | `tune` 을 SSH 에서 대화형으로 (`hard_cap`·`grip_util`) — 맥북 SSH 작업에 유용 |
| `1ffd3df` | 주행 종료 시 CSV·bag 을 car1/car2 브랜치에 자동 커밋·푸시 |
| `1305888` | bag 에 `/frenet/*` 를 녹화 — 회피 결과를 사후 판정할 수 있게 |
| `6dbc9f6` | `.gitignore` — 주행 bag 2.0GB 가 커밋 대기 중이었다 |
| `f20fdff` | `docs/실행어_모음집.md` — `.sh` 전부를 한 장에 |
| `ce7a646` | 브랜치 규칙을 실제 운영(car1/car2)에 맞게 갱신 |

## 1대1 (지금 당장은 아니지만 대회 전 필요)

`4417363`(`./race.sh race` 동적 장애물 추적·예측 회피), `078764c`/`1ed18f1`/`9ae6904`
(추월 가능 구간), `511ce12`(상대 예측을 레이싱라인 s 를 따라), `2fa6dca`(앞차에 맞은
빔을 AMCL 입력에서 제거), `364bf9b`(AMCL 어긋남만큼 검출 문턱 자동 조정),
`070fbbb`/`69d481c`/`1b6dd9a`.

---

## ⚠ 머지할 때 주의

`race_script.yaml` 에서 충돌이 날 것이다. car2 쪽에만 있는 값을 지키자:

```
map_name / waypoints        0815_clean / 0815_clean_wp_kappa
localization.init_x/y/yaw   5.2699 / -0.3671 / -0.1796   (0815 웨이포인트 140)
mission.start_at_init       true  ← car2 에서 신설. main 에는 없다
frenet.clearance / d_max    car2 튜닝값
frenet.per_controller.map   호라이즌 (2.6, 1.0, 6.0)
```

`config/vehicle_car2.yaml` 은 car2 것이라 그대로 두면 된다 — 특히
`steering_angle_to_servo_gain: -0.89` 는 주행 로그로 검증한 값이다.

`mission.start_at_init` 은 car2 에서 만든 기능이라 **main 에 없다.**
`race.launch.py` 를 main 쪽으로 덮어쓰면 사라지니 확인할 것
(기본값이 `true` 라 키가 없어도 동작은 유지된다).
