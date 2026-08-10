# yax — RoboRacer 2026 F1TENTH 저장소

---

## ⚠️ 두 대의 차가 이 저장소를 함께 씁니다 — 작업 전 반드시 읽을 것

**브랜치는 `main` 하나입니다. 코드는 합치고 값만 분리합니다.** (2026-08-10 전환)

예전엔 차마다 브랜치를 나눴는데, 그러면 **코드까지 갈라져 같은 버그를 두 번**
고치게 됩니다. 실제로 그랬습니다 — `bench(box)` 모드가 무력화된 걸 두 차가
따로 디버깅해 각자 커밋했습니다. 그래서 값만 파일로 분리했습니다.

### 첫 실행 전에 한 번 — 이 젯슨이 몇 호차인지 알려주기

```bash
echo car2 > .car        # 1호차면 car1
```

`.car` 는 `.gitignore` 라 커밋되지 않습니다. 기기마다 한 번만 하면 됩니다.

### 구조

```
config/vesc_base.yaml       배관(포트·스무더·리밋). 두 차 공용, git
config/vehicle_car1.yaml    1호차가 재서 나온 값,      git
config/vehicle_car2.yaml    2호차,                     git
.car                        "car2" 한 줄               .gitignore
        ↓ config/apply_vehicle.py  (race.sh 가 매 실행 호출)
config/vesc.yaml                생성물  .gitignore
config/.vehicle_overlay.yaml    race_script.yaml 위에 덮을 값  .gitignore
```

**`config/vesc.yaml` 은 git 에 없습니다.** 차량 캘리브레이션을 잘못 커밋할
방법 자체가 없어졌습니다.

### 무엇이 차량별인가

| 절 | 키 |
|---|---|
| `vesc` | `speed_to_erpm_gain`, 조향 `gain`/`offset`, `servo_min`/`max` |
| `geometry` | `wheelbase`, `mass`, `lf`, `lr` |
| `tire` | `mu`, `pac_b/c/d/e` |
| `tuning` | `l1_min`, `l1_k`, `steer_alpha`, `slip_gain`, `speed_lag`, `a_lon_*`, `grip_util` |
| `joystick` | `steer_invert`, `speed_invert`, `device`, `axis_*`, `btn_*` |
| `network` | `peers` (젯슨 IP — 1호차 `.57` / 2호차 `.77`) |

`race_script.yaml` 의 나머지(알고리즘·안전·회피·계측)는 **그대로 공용**입니다.
이게 핵심입니다 — 한쪽에서 고친 알고리즘이 자동으로 양쪽에 흐릅니다.

`max_steer` 는 **손으로 적지 않습니다.** `(servo_min − offset) / (−gain)` 으로
`apply_vehicle.py` 가 계산해 내려보냅니다. 따로 적으면 반드시 어긋납니다.

### 규칙

- **남의 차 파일(`vehicle_car1.yaml` ↔ `vehicle_car2.yaml`)은 건드리지 마세요.**
  각자 실측값입니다
- 코드·알고리즘 수정은 **`main` 에 바로 올리세요** — 그래야 양쪽에 흐릅니다
- 기동 로그의 `[vehicle]` 줄을 매번 확인하세요. `⚠` 가 뜨면 이유가 있습니다
- 자세한 절차와 캘리브레이션 순서: `docs/car2_merge_guide.md`

### ⚠ 아직 안 나눈 것 — 트랙 설정

`map_name` / `waypoints` / `mission.start_*` / `localization.init_*` 는
`race_script.yaml` 에 있고 **두 차가 공유**합니다. 두 차가 다른 트랙을 돌면
부딪히니, 당장은 **커밋하지 말고 로컬 수정으로** 두세요.

---

## 저장소 정보

- **원격:** `git@github.com:lloes098/26-jetson-f1tenth.git`
- **푸시 브랜치:** `main` (구 저장소 `naykwak/f1TENTH`의 `hj_v2_f1tenth_ws`는 더 이상 사용 안 함)
- **젯슨 내 경로:** `~/yax` — 이 git repo가 **유일한 소스**

## 두 개의 스택

| 스택 | 위치 | 용도 | 문서 |
|------|------|------|------|
| 실차(젯슨) race_stack | `src/` (race_stack, q2_gap_follow, roboracer_1v1) | 실제 차량 자율주행 (대회 본선) | `JETSON_COMMANDS.md`, `README.md` |
| 시뮬 번들 | `f1TENTH-contest_full_stack/` | f1tenth_gym_ros 시뮬 + 컨트롤러 개발 | 해당 폴더의 `CLAUDE.md`, `README.md` |

## 심링크 구조 (중요)

실행 워크스페이스 `~/f1tenth_ws/src/`의 race_stack·q2_gap_follow·roboracer_1v1은
이 repo의 `src/`로 향하는 **심링크**다 (복사 아님, `install_on_jetson.sh`가 생성).

- repo에서 파이썬 코드 수정 → 재빌드 없이 즉시 반영 (`--symlink-install`)
- `~/f1tenth_ws` 쪽에서 수정해도 그게 곧 repo → 커밋/푸시로 보존
- 재빌드가 필요한 경우: 새 `.py`/launch 파일 추가, `setup.py`/`package.xml` 변경
  → `cd ~/f1tenth_ws && colcon build --packages-select race_stack q2_gap_follow roboracer_1v1 --symlink-install`
- **맵/웨이포인트 신규 추가는 재빌드 불필요** (2026-07-17~): `race.sh`가 `RACE_WS`를
  export하고, `gen_waypoints`·`race.launch.py`가 이 값 기준으로 repo `src/race_stack`의
  `maps/`·`waypoints/`를 직접 읽는다. 예전엔 build/install 스냅샷을 읽어 새 맵이 안 보였음.
  (그래도 새 맵 파일은 커밋/푸시로 repo에 보존할 것)
- `race_script.yaml`은 repo(`~/yax`)에만 존재 — `race.sh`가 자기 디렉토리에서 읽음
- `config/vesc.yaml`(조향/모터 캘리브레이션)도 repo가 원본 — `~/f1tenth_ws`의
  f1tenth_stack src·install 양쪽이 여기로 심링크됨. f1tenth_stack을 colcon build
  하면 install 링크가 복사본으로 덮이니 `install_on_jetson.sh` 재실행으로 재링크

## 주요 스크립트

- `race.sh` — 실차 딸깍 실행 (mapping/q1/q2/q3/race/savemap/log/waypoints/joy/teleop)
- `jetson.sh` — 노트북에서 SSH+tmux로 젯슨의 race.sh 원격 실행
- `install_on_jetson.sh` — 최초 1회 설치 (심링크 + 빌드 + udev)
- `race_script.yaml` — ★ 팀원이 수정하는 유일한 설정 파일

## 환경 주의사항

- **numpy는 1.26.4 고정** — 2.x로 올리면 cv2/matplotlib/colcon이 깨짐 (`~/.local`에 pyparsing도 설치 금지)
- ROS 2 Humble / Ubuntu 22.04 (Jetson, aarch64)
- 조이스틱 트리거: `trigger_invert`는 `/joy` 토픽 기준 — RT 중립값 부호가 js0 원시값과 반대
