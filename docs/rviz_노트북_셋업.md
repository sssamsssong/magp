# 실시간 RViz 관찰 — 노트북 셋업 가이드

> 작성일: 2026-07-17
> 젯슨 주행을 노트북에서 실시간(맵+레이싱라인+차 위치/궤적+LiDAR+목표점)으로 보기 위한 정리.

## 이번에 추가된 것 (이미 main 에 커밋/푸시됨)

| 커밋 | 내용 |
|------|------|
| `491dc8c` | gen_waypoints 웨이포인트 생성 시 맵 오버레이 PNG 자동 저장 (`waypoints/<wp>.png`) |
| `c111371` | 주행 중 실시간 RViz 시각화 — stanley_controller 시각화 토픽 발행 + `config/race.rviz` + jetson.sh 자동기동 |
| `e0eecdd` | `rviz.sh` — 직접 SSH 워크플로우용 노트북 RViz 뷰어 |

### 젯슨 주행 노드가 항상 발행하는 시각화 토픽
- `/waypoints_path` — 전체 레이싱라인 (속도 컬러, 1Hz 재발행)
- `/car_path` — 차가 지나온 궤적
- `/waypoint` — 현재 목표점 / `/obstacle` — Q2 장애물
- `/map`, `/scan`, `/amcl_pose` (+ TF) — 맵·라이다·차 위치
> viz 값과 무관하게 발행되므로, **노트북에서 뷰어만 켜면 된다.**

---

## 노트북에서 할 일 (우리 방식 = 직접 SSH)

### A. 최초 1회 준비
1. **ROS 2 Humble + rviz2 설치**
   ```bash
   sudo apt install ros-humble-rviz2      # 또는 ros-humble-desktop
   ```
2. **이 repo 를 노트북에 클론** (없으면)
   ```bash
   git clone git@github.com:lloes098/26-jetson-f1tenth.git
   ```
   이미 있으면:
   ```bash
   cd <repo 폴더> && git pull
   ```
   → `rviz.sh` 와 `config/race.rviz` 를 노트북에서 읽는다.
3. **네트워크 / 도메인 확인**
   - 노트북과 젯슨이 같은 네트워크(`192.168.45.x`)
   - `ROS_DOMAIN_ID` 가 양쪽 동일해야 함 (기본 미설정=0).
     노트북 `~/.bashrc` 에서 도메인을 바꿔놨다면 젯슨과 같은 값으로.

### B. 매번 주행할 때 (터미널 2개)
**① 젯슨 터미널 (SSH):**
```bash
ssh jetson@192.168.45.246
cd ~/yax && ./race.sh q2        # 평소대로 주행 (q1/q2/q3/race)
```

**② 노트북 로컬 터미널 (SSH 아님):**
```bash
cd <repo 폴더> && ./rviz.sh      # RViz 뷰어 실행
```
`rviz.sh` 가 ROS 소싱 → `config/race.rviz` 로 rviz2 실행. rviz2 미설치·
토픽 미수신이면 안내 메시지를 띄운다 (주행 시작하면 자동으로 나타남).

---

## 문제 해결
- **RViz 는 떴는데 아무것도 안 보임** → 젯슨에서 race.sh 주행이 실제로 돌고
  있는지, 그리고 노트북·젯슨의 네트워크/`ROS_DOMAIN_ID` 가 같은지 확인.
  노트북 터미널에서 `ros2 topic list | grep waypoints_path` 로 토픽 수신 점검.
- **차 위치(빨간 화살표)가 안 움직임** → AMCL 수렴 전이거나 로컬라이제이션
  문제. RViz 의 2D Pose Estimate 로 초기 위치를 잡아줄 수도 있음.
- **rviz2 없음** → `sudo apt install ros-humble-rviz2`

## 참고
- `jetson.sh` (노트북에서 원격 실행) + `race_script.yaml` 의 `viz: true` 는
  **jetson.sh 를 쓸 때만** RViz 를 자동 기동한다. 우리처럼 직접 SSH 로
  race.sh 를 돌리는 경우엔 `rviz.sh` 를 쓰면 되고 viz 값은 영향 없다.
