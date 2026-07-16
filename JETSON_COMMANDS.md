# 젯슨 실행 명령어 총정리

저장소: `main` — https://github.com/lloes098/26-jetson-f1tenth
젯슨 내 경로: `~/yax` (git repo가 유일 소스. `~/f1tenth_ws/src/`의 race_stack 등 3개 패키지는 여기로 심링크됨)

---

## 0. SSH 연결 (노트북 → 젯슨)

```bash
ssh jetson@192.168.45.246
```

> 비밀번호 없이 접속하려면 (선택사항):

```bash
# 노트북에서 한 번만 실행
ssh-keygen -t ed25519
ssh-copy-id jetson@192.168.45.246
```

> 노트북에 이 repo를 클론했다면 `./jetson.sh q1|q2|savemap <이름>|log <이름>|stop|status|attach` 로
> SSH+tmux를 통해 젯슨의 `race.sh`를 원격 실행할 수도 있음.

---

## 1. 폴더 이동 (한 번만)

```bash
# 젯슨에 이미 클론돼 있음
cd ~/yax

# 새로 클론해야 하는 경우
git clone git@github.com:lloes098/26-jetson-f1tenth.git ~/yax
cd ~/yax

# 실행 권한 부여 (필요 시)
chmod +x install_on_jetson.sh race.sh
```

---

## 2. 설치 (딱 한 번만)

```bash
./install_on_jetson.sh
```

> `~/f1tenth_ws`가 이미 있고 `f1tenth_stack`이 빌드된 상태여야 합니다.
> 이 스크립트는 `~/yax/src/`의 패키지 3개(race_stack, q2_gap_follow, roboracer_1v1)를
> `~/f1tenth_ws/src/`에 **심링크**로 연결(복사 아님)하고 colcon build 합니다.
> 이후 repo에서 파이썬 코드를 수정하면 재빌드 없이 즉시 반영됩니다(`--symlink-install`).

---

## 3. VESC 연결 확인

```bash
# VESC USB 꽂혀 있는지 확인
ls /dev/ttyACM*

# 심링크 확인
ls -l /dev/sensors/vesc
```

> 심링크가 없어도 `race.sh`가 실행 시 자동 생성하므로 보통 신경 쓸 필요 없음.

---

## 4. 맵 만들기 (트랙 처음 갈 때)

```bash
# SLAM + 조이스틱 (추천)
./race.sh mapping-joy
# RT로 트랙 1~2바퀴 천천히 주행 → Ctrl+C

# 또는 SLAM + 키보드
./race.sh mapping
# w/a/s/d 로 트랙 1~2바퀴 천천히 주행 → Ctrl+C

# 맵 저장
./race.sh savemap my_track
```

> `mapping-joy`는 bringup + SLAM + joystick_teleop을 한 번에 기동함.
> `joystick.launch.py`와 동시에 쓰면 bringup 중복으로 충돌 — 함께 쓰지 말 것.
> 수동조작 속도 상한은 `race_script.yaml`의 `speed.mapping_max`로 조절 (현재 1.0 m/s).

---

## 4-1. 맵 정제 + 경로 생성 (SLAM 맵이 지저분할 때)

> SLAM으로 찍은 raw 맵은 레이저 노이즈가 많아 그대로 쓰기 어려움.  
> 아래 두 단계로 깨끗한 맵 + 최적화 경로를 만들어 `race_script.yaml`에 등록.  
> 젯슨에 모니터 연결 후 직접 실행.

### 사전 설치 (처음 한 번만)

```bash
sudo apt install python3-tk
```

### Step 1 — 맵 정제

```bash
cd ~/yax/src/race_stack/maps
python3 clean_map.py
# → 터미널에서 맵 목록이 나타남. 번호 또는 이름 입력으로 선택.
#   (* 표시 = 이미 _clean 버전이 있는 맵)
#   출력 파일은 <맵이름>_clean 으로 자동 결정.
#
# 맵/출력 이름을 직접 지정하려면:
# python3 clean_map.py --map <맵이름> --out <출력이름>
```

| 키 | 동작 |
|----|------|
| `1` | 외부 바운더리 모드 (빨강) — 트랙 방 전체 테두리 |
| `2` | 내부 바운더리 모드 (파랑) — 중앙 내벽 |
| 좌클릭 | 꼭짓점 추가 (직선 연결) |
| 우클릭 / `z` | 마지막 꼭짓점 취소 |
| `Enter` | 마스크 적용 (폴리곤 안만 남기고 바깥 노이즈 전부 제거) |
| `c` | closing 반복 횟수 토글 (0 → 2 → 4) |
| `s` | 저장 (`<출력이름>.pgm` + `.yaml`) |
| `r` | 리셋 |
| `q` | 종료 |

> **주의**: 외부 + 내부 폴리곤 둘 다 그려야 트랙 코리도가 정확히 분리됨.  
> 외부만 그리면 내벽이 없어 경로가 내벽을 통과할 수 있음.

### Step 2 — 경로 생성

> 반드시 정제된 맵 사용. raw SLAM 맵은 복도 전체가 연결되어 스켈레톤 추출 실패.

```bash
# 방법 1 — race.sh로 실행 (권장)
cd ~/yax
./race.sh waypoints              # 맵 목록에서 선택
./race.sh waypoints track_2_clean  # 맵 이름 직접 지정

# 방법 2 — 직접 실행 (옵션 세밀 조정 시)
cd ~/yax/src/race_stack/race_stack
python3 gen_waypoints.py
# python3 gen_waypoints.py --map track_2_clean --safety 0.2 --vmax 4.0 --vmin 1.0
```

| 단계 | 동작 |
|------|------|
| 맵 선택 | 터미널에서 번호 또는 이름 입력 |
| 시드 클릭 | 트랙 **내부**를 클릭 → Enter (잘못 된 영역 선택 방지) |
| 후보 경로 | 스켈레톤 기반 후보들이 색깔로 표시됨 |
| 좌클릭 | 원하는 경로 위에 점 몇 개 추가 |
| `Enter` | 후보 매칭 + 랩타임 최적화 + 자동 저장 |
| `r` / `s` | 리셋 / 재저장 |

**저장 위치**
```
waypoints/<맵이름>_wp.csv          ← Stanley / Pure Pursuit 입력 (x, y, speed)
maps/<맵이름>/inner_bound.npy      ← Q2 장애물 회피용 경계
maps/<맵이름>/outer_bound.npy
```

### Step 3 — race_script.yaml 수정

```bash
nano ~/yax/race_script.yaml
```

```yaml
map_name: track_2_clean      # 정제된 맵 이름
waypoints: track_2_clean_wp  # 생성된 웨이포인트 이름
controller: stanley
mode: q1

localization:
  init_x: 0.0    # 차를 놓는 실제 위치 (맵 좌표계 기준)
  init_y: 0.0    # 픽셀 → 월드: x = px*0.05 + origin_x
  init_yaw: 0.0  # 차가 향하는 방향 (rad)
```

---

## 5. 웨이포인트 기록 (레이싱라인)

`waypoint_logger`는 `/amcl_pose`를 구독하므로 **AMCL이 켜진 race 스택이 먼저 떠 있어야** 함.

### 방법 A — gap_follower가 자동으로 달리면서 기록 (빠른 방법)

```bash
# 터미널 1: race 스택 기동 (AMCL + gap_follower)
./race.sh q2

# 터미널 2: 웨이포인트 기록
./race.sh log my_track_wp
# LB 눌러 자율모드 → gap_follower가 1~2바퀴 자동 주행 → Ctrl+C
```

### 방법 B — 수동으로 깨끗한 레이싱라인 기록

`./race.sh q2`는 joystick 노드를 포함하지 않아 수동 조종 불가.
AMCL만 필요하므로 별도 터미널에서 localization 노드를 따로 띄워야 함:

```bash
# 터미널 1: bringup + AMCL + gap_follower
./race.sh q2   # AMCL 확보 목적, 실제 주행은 수동으로

# 터미널 2: joystick_teleop만 단독 실행 (joy_node 먼저)
ros2 run joy joy_node
# 터미널 3
ros2 run race_stack joystick_teleop --ros-args -p max_speed:=1.5

# 터미널 4: 기록
./race.sh log my_track_wp
# 수동으로 한 바퀴 깨끗하게 돌기 → Ctrl+C
```

---

## 6. race_script.yaml 수정

```bash
nano ~/yax/race_script.yaml
```

> `race.sh`는 자기가 있는 디렉토리(`~/yax`)의 `race_script.yaml`을 읽음. `~/f1tenth_ws`에는 이 파일이 없음.

```yaml
map_name: my_track       # savemap 때 쓴 이름
waypoints: my_track_wp   # log 때 쓴 이름
controller: stanley      # stanley / pure_pursuit / gap_follower
```

---

## 7. 자율주행 실행

```bash
./race.sh q1      # 장애물 없음 + 레이싱라인 추종
./race.sh q2      # 정적 장애물 회피 (gap follower)
./race.sh q3      # 장애물 없음 + 공격적 속도 (2분 제한)
./race.sh race    # 1v1 (상대 추적 + 추월)
```

---

## 8. 수동 조종

```bash
./race.sh joy     # 조이스틱 (LB 버튼 = 수동 ↔ 자율 토글)
./race.sh teleop  # 키보드 (w/a/s/d)
```

---

## 9. 비상 정지

```bash
ros2 topic pub --once /race/enabled std_msgs/msg/Bool "{data: false}"
```

---

## 10. 트러블슈팅

```bash
# VESC 심링크 수동 생성 (race.sh가 자동 생성하지만, 직접 만들 때)
sudo mkdir -p /dev/sensors
sudo ln -sf /dev/ttyACM0 /dev/sensors/vesc

# ROS 토픽 정상 발행 확인
ros2 topic list
ros2 topic echo /amcl_pose     # 위치 추정 확인
ros2 topic echo /race/enabled  # 미션 매니저 상태

# 빌드 다시 — 파이썬 코드만 고쳤으면 불필요 (--symlink-install 이라 즉시 반영).
# 새 파일 추가 / setup.py·package.xml 변경 / launch·config 추가 때만 실행:
cd ~/f1tenth_ws
colcon build --packages-select race_stack q2_gap_follow roboracer_1v1 --symlink-install
source install/setup.bash
```

---

## 11. VESC Tool 원격 설정 (노트북에서 VESC 설정 변경)

VESC가 젯슨 USB에 꽂혀 있을 때, 노트북의 VESC Tool로 원격 설정하는 방법입니다.

```
[노트북] --- SSH/네트워크 --- [젯슨] --- USB --- [VESC]
  VESC Tool 실행                  USB/IP 서버
```

> **주의**: VESC Tool 사용 중에는 `race.sh`를 꺼야 합니다. 동시에 실행하면 UART 충돌이 납니다.

### 젯슨에서 (USB/IP 서버 설정)

```bash
# 패키지 설치
sudo apt-get install -y usbip linux-tools-generic

# 커널 모듈 로드
sudo modprobe usbip-core
sudo modprobe usbip-host

# VESC 장치 busid 확인
usbip list -l
# 출력 예시: - busid 1-2.1 (0483:5740)  ← VESC

# USB/IP 데몬 시작 + VESC 공유
sudo usbipd -D
sudo usbip bind -b 1-2.1   # busid는 위에서 확인한 값으로
```

### 노트북에서 — Linux

```bash
sudo apt-get install -y usbip
sudo modprobe vhci-hcd

# 젯슨에서 공유 중인 장치 확인
usbip list -r 192.168.45.246

# VESC 연결
sudo usbip attach -r 192.168.45.246 -b 1-2.1

# 연결 확인
ls /dev/ttyACM*
```

### 노트북에서 — Windows

```powershell
# 관리자 PowerShell에서
winget install usbipd
usbipd attach --remote 192.168.45.246 --busid 1-2.1
# 장치 관리자에서 COMx 포트 확인 후 VESC Tool에서 해당 포트 선택
```

### VESC Tool 연결

```
1. VESC Tool 실행
2. 우측 상단 플러그 아이콘 클릭
3. 포트 선택: /dev/ttyACM0 (Linux) 또는 COMx (Windows)
4. Connect
```

### 작업 끝난 후 연결 해제

```bash
# 노트북에서
sudo usbip detach -p 00   # 포트번호: usbip port 명령으로 확인

# 젯슨에서
sudo usbip unbind -b 1-2.1
```

---

## 12. VNC — 젯슨 화면 원격 조작 (인터넷 없이 LAN만으로 가능)

```
[공유기]
   ├── 노트북 (Linux)
   └── 젯슨  (192.168.45.246)
```

### 최초 1회 — 패키지 설치 및 비밀번호 설정 (젯슨에서)

```bash
sudo apt-get install -y xvfb x11vnc openbox

# VNC 비밀번호 설정
x11vnc -storepasswd

# 자동 시작 스크립트 생성
cat > ~/start_vnc.sh << 'EOF'
#!/usr/bin/env bash
Xvfb :1 -screen 0 1920x1080x24 &
sleep 1
DISPLAY=:1 openbox &
sleep 1
x11vnc -display :1 -forever -noxdamage -rfbauth ~/.vnc/passwd -rfbport 5900 -shared -bg
EOF
chmod +x ~/start_vnc.sh
```

---

### 매번 VNC 연결하는 방법

#### 젯슨에서 (SSH 접속 후) — 한 줄씩 순서대로

```bash
Xvfb :1 -screen 0 1920x1080x24 &
```
2~3초 기다린 후:
```bash
DISPLAY=:1 openbox &
```
2~3초 기다린 후:
```bash
x11vnc -display :1 -forever -noxdamage -rfbauth ~/.vnc/passwd -rfbport 5900 -shared
```

> 마지막 줄은 반드시 **한 줄**로 붙여넣기. 줄 바꿈 들어가면 에러납니다.

#### 노트북(Linux)에서

```bash
vncviewer 192.168.45.246:5900
```

---

### VNC 재시작 (충돌 났을 때)

이전 프로세스가 남아있으면 충돌이 납니다. 이때는 다 죽이고 다시 시작:

```bash
pkill x11vnc; pkill openbox; pkill Xvfb
```
잠깐 기다린 후 위의 매번 VNC 연결 순서 다시 실행.

---

### VNC 종료

```bash
pkill x11vnc; pkill openbox; pkill Xvfb
```

---

## 13. race.sh vs 수동 실행 비교

`race.sh`는 아래 수동 명령들을 한 줄로 대체함.

| 단계 | 수동 방법 | race.sh |
|------|-----------|---------|
| bringup | 터미널1에서 직접 launch | 자동 포함 |
| SLAM | 터미널2에서 직접 launch | 자동 포함 |
| 조종 | 터미널4에서 별도 스크립트 | 자동 포함 |
| 맵 저장 위치 | `~/maps/` (race stack이 못 읽음) | `~/yax/src/race_stack/maps/` (= `~/f1tenth_ws/src/race_stack/maps/`, 바로 사용 가능) |

---

## 14. 전체 실행 순서 (race.sh 기준)

### 1단계 — 맵 따기

```bash
ssh jetson@192.168.45.246
cd ~/yax   # race.sh 있는 경로

# 조이스틱으로 맵핑 (bringup + SLAM + joystick 한번에)
./race.sh mapping-joy
# RT로 트랙 1~2바퀴 천천히 주행 → Ctrl+C
```

```bash
# 다른 터미널에서 맵 저장
./race.sh savemap my_track
```

### 2단계 — 경로(웨이포인트) 따기

```bash
# 터미널 1: race 스택 기동 (AMCL 포함)
./race.sh q2

# 터미널 2: 경로 기록
./race.sh log my_track_wp
# LB 눌러 자율모드 → gap_follower가 달리면서 기록 → Ctrl+C
```

### 3단계 — 자율주행 테스트

```bash
# race_script.yaml이 아래처럼 설정돼 있어야 함
# map_name: my_track
# waypoints: my_track_wp
# controller: stanley

./race.sh q1   # Stanley로 자율주행
```

> **git pull 후**: 패키지가 `~/f1tenth_ws/src/`에 **심링크**로 연결돼 있어 pull만 하면 즉시 반영됨
> (`race.sh`, `race_script.yaml` 포함). 재빌드가 필요한 경우는 새 파일 추가·setup.py 변경 등뿐 —
> 그때만 `cd ~/f1tenth_ws && colcon build --packages-select race_stack q2_gap_follow roboracer_1v1 --symlink-install`.
> `./install_on_jetson.sh` 재실행은 새 의존 패키지가 추가됐을 때만 필요.

---

## 15. AMCL 로컬라이제이션 상세

### 동작 원리

```
LiDAR(/scan) ──┐
               ├─► AMCL ─► /amcl_pose (현재 위치) ─► Stanley 컨트롤러
odom(/odom) ───┘
map_server(/map) ─► AMCL (매칭 기준 맵)
```

AMCL은 **파티클 필터** 기반 위치 추정기.  
초기 위치 주변에 파티클(위치 가설) 수백 개를 뿌리고,  
LiDAR 스캔이 저장된 맵과 얼마나 일치하는지 점수를 매겨 수렴시킴.  
`./race.sh q1` 실행 시 `map_server + AMCL + lifecycle_manager`가 자동으로 뜸.

---

### 핵심: 초기 위치(init_x / init_y / init_yaw) 잡는 법

**초기 위치가 실제 차 위치와 많이 다르면 파티클이 수렴하지 못해 주행 불가.**

#### 1단계 — 픽셀 좌표 → 월드 좌표 변환

`track_2_clean.yaml` 기준:
- `resolution: 0.05` m/px
- `origin: [-6.33, -7.88]` (맵 이미지 **좌하단** 꼭짓점의 월드 좌표)
- 맵 크기: 272 × 206 px

```
world_x = pixel_col        × 0.05 + (-6.33)
world_y = (206 - pixel_row) × 0.05 + (-7.88)
```

> 맵 이미지에서 차를 놓을 위치의 픽셀 좌표(col, row)를 읽어 위 공식으로 변환.  
> 이미지 뷰어(GIMP, eog 등)로 `track_2_clean.pgm`을 열면 마우스 위치의 픽셀 좌표를 알 수 있음.

**현재 track_2_clean 좌표 범위**
```
X: -6.33 ~ 7.27 m
Y: -7.88 ~ 2.42 m
트랙(자유공간) 중심 부근: (1.76, -0.19) m
```

#### 2단계 — init_yaw (차 방향) 설정

ROS 좌표계: 오른쪽(+x 방향) = 0 rad, 반시계 방향이 양수.

| 차가 향하는 방향 | init_yaw |
|-----------------|---------|
| +x (오른쪽) | 0.0 |
| +y (위쪽) | 1.571 (π/2) |
| -x (왼쪽) | 3.142 (π) 또는 -3.142 |
| -y (아래쪽) | -1.571 (-π/2) |

> 트랙 첫 번째 웨이포인트 기준 주행 방향: **-152도 ≈ -2.66 rad** (대략 왼쪽-아래 방향)  
> 차를 첫 웨이포인트 근처에 놓고 그 방향으로 맞추면 됨.

#### 3단계 — race_script.yaml 수정

```bash
nano ~/yax/race_script.yaml
```

```yaml
localization:
  init_x: 0.88    # 예시: 첫 웨이포인트 근처
  init_y: 0.37
  init_yaw: -2.66  # 차가 향하는 방향 (rad)
```

> **팁**: 초기 위치를 정확히 몰라도 ±0.5m 오차 정도면 AMCL이 수렴함.  
> 완전히 모를 경우 트랙 자유공간 중심 (1.76, -0.19) + 적당한 yaw로 시작해도 됨.

---

### 실행 및 수렴 확인

```bash
# 자율주행 스택 실행 (AMCL 포함)
./race.sh q1
```

**다른 터미널에서 수렴 여부 확인:**

```bash
# 1) 위치 추정값 출력 (숫자가 안정되면 수렴)
ros2 topic echo /amcl_pose --once

# 2) 공분산 확인 (작을수록 수렴)
ros2 topic echo /amcl_pose 2>/dev/null | grep -A3 "covariance"

# 3) 파티클 분포 확인 (수렴 시 하나의 점으로 모임)
ros2 topic echo /particle_cloud --once | head -20

# 4) TF 브로드캐스트 확인 (map→odom 변환이 있어야 주행 가능)
ros2 run tf2_ros tf2_echo map odom
```

**수렴 판정 기준:**
- `/amcl_pose` x, y, yaw 값이 거의 고정되면 수렴
- covariance 앞 두 값(xx, yy)이 0.01 이하이면 양호
- `tf2_echo map odom`에서 translation이 안정되면 OK

---

### AMCL 수렴이 안 될 때 체크리스트

| 증상 | 원인 | 해결 |
|------|------|------|
| `/amcl_pose`가 아예 안 들어옴 | lifecycle 노드 미기동 or LiDAR 문제 | `ros2 topic list`에 `/scan` 있는지 확인 |
| 위치가 계속 흔들림 | 초기 위치가 많이 틀렸거나 맵 불일치 | init_x/y 재조정 후 재시작 |
| 위치가 완전히 엉뚱한 곳 | 맵에서 유사한 구조가 있어 파티클이 거기로 수렴 | init_yaw 포함 초기 위치 더 정확하게 |
| 잠깐 수렴했다가 다시 흔들림 | 차가 너무 빨리 출발 (AMCL 업데이트 전) | `race.launch.py`의 컨트롤러 지연 3초 이미 설정됨 |
| 맵은 잘 뜨는데 `/amcl_pose` 없음 | lifecycle_manager가 AMCL 활성화 실패 | 로그에서 lifecycle 오류 확인 |

```bash
# 토픽 정상 여부 한번에 확인
ros2 topic list | grep -E "scan|amcl|map|odom"

# 예상 출력:
# /amcl_pose
# /map
# /odom
# /particle_cloud
# /scan
```

---

### AMCL 초기 포즈 강제 재설정 (수렴 실패 시)

```bash
# 위치만 재설정 (차를 다시 제자리에 놓은 후)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.88, y: 0.37, z: 0.0}, orientation: {z: -0.936, w: 0.352}}}}"
# orientation: yaw=-2.66rad → z=sin(-2.66/2)=-0.936, w=cos(-2.66/2)=0.352
```

> 또는 노트북에서 RViz2 실행 후 **2D Pose Estimate** 버튼으로 클릭 설정 가능:
> ```bash
> # 노트북에서 (젯슨 ROS_DOMAIN_ID 같게 설정 후)
> rviz2
> # Add → By topic → /map 추가, 2D Pose Estimate 클릭 후 맵에서 위치 지정
> ```

---

## 순서 요약

### 방법 A — gap_follower로 자동 웨이포인트 기록 (빠름)
```
설치 → VESC 확인 → 맵핑(mapping-joy) → 맵 저장(savemap) → 웨이포인트 기록(log) → yaml 확인 → 자율주행(q1)
```

### 방법 B — 맵 정제 + 최적 경로 생성 (맵이 지저분할 때)
```
설치(python3-tk) → VESC 확인 → 맵핑(mapping-joy) → 맵 저장(savemap)
  → [젯슨 모니터] clean_map.py (맵 정제)
  → [젯슨 모니터] gen_waypoints.py (최적 경로 생성)
  → yaml 수정 (map_name, waypoints, init 위치)
  → 자율주행(q1)
```
