# yax — RoboRacer 2026 F1TENTH 저장소

---

## ⚠️ 두 번째 차량(car2)에서 작업 중이라면 — push 전 반드시 읽을 것

**main 브랜치에 직접 push하지 마세요.**

car1(1호차)과 car2(2호차)가 같은 저장소를 공유합니다.
car2의 근본 세팅(VESC 캘리브레이션, 조향 파라미터 등)을 main에 바로 올리면
**car1 설정이 덮여 대회 주행이 망가질 수 있습니다.**

### 해야 할 것

```bash
# 브랜치 생성 후 작업
git checkout -b car2/작업내용   # 예: car2/vesc-calib, car2/steering-fix
git push -u origin car2/작업내용
```

브랜치에 올린 뒤 팀장(hj)에게 알려주세요. main 머지는 팀장이 검토 후 합니다.

### main에 올려도 되는 것

- 문서(docs/)
- 두 차량 공통으로 검증된 코드 변경 (팀장 확인 후)

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
