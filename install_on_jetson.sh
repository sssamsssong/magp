#!/usr/bin/env bash
# ============================================================
# install_on_jetson.sh — 젯슨에 race 패키지 설치 (한 번만 실행)
#
# 전제: f1tenth_ws가 이미 존재하고 f1tenth_stack이 빌드되어 있음
#       (기존에 bringup_launch.py 돌리던 그 환경)
# ============================================================
set -e
WS=${RACE_WS:-$HOME/f1tenth_ws}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[1/4] 의존 패키지 설치"
sudo apt-get update
sudo apt-get install -y \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-slam-toolbox \
    ros-humble-ackermann-msgs \
    ros-humble-joy \
    ros-humble-foxglove-bridge \
    python3-yaml \
    python3-opencv \
    python3-scipy \
    python3-matplotlib \
    python3-networkx \
    python3-pil

# scikit-image (gen_waypoints용), scikit-learn (opponent tracker DBSCAN용)
# numpy는 1.26.4 고정 — 2.x로 올라가면 cv2/matplotlib/colcon 이 깨진다
pip3 install "numpy==1.26.4" scikit-image scikit-learn --quiet

echo "[2/4] 패키지 심링크 → $WS/src/  (복사 아님: 이 git repo가 유일 소스)"
# cp 가 아니라 심링크. 실행 WS 는 git repo 를 그대로 가리키므로
# repo 에서 편집하면 즉시 반영되고(파이썬은 --symlink-install 로 무빌드),
# 실행 WS 쪽에서 고쳐도 그게 곧 git repo 라 커밋/푸시로 보존된다.
mkdir -p "$WS/src"
ln -sfn "$SCRIPT_DIR/src/race_stack"    "$WS/src/race_stack"
ln -sfn "$SCRIPT_DIR/src/q2_gap_follow" "$WS/src/q2_gap_follow"
ln -sfn "$SCRIPT_DIR/src/roboracer_1v1" "$WS/src/roboracer_1v1"
ln -sfn "$SCRIPT_DIR/race.sh"           "$WS/race.sh"   # jetson.sh 가 WS 에서 ./race.sh 호출

# vesc.yaml (조향/모터 캘리브레이션)도 repo 를 원본으로 심링크.
# f1tenth_stack 은 이 repo 밖 패키지지만, 캘리브레이션 값은 git 으로
# 보존해야 하므로 src 와 install 양쪽을 repo/config/vesc.yaml 로 건다.
# ★ f1tenth_stack 을 colcon build 하면 install 쪽 심링크가 복사본으로
#   덮이므로, 그 뒤엔 이 스크립트를 다시 실행해 재링크할 것.
F1T_SRC="$WS/src/f1tenth_system/f1tenth_stack/config/vesc.yaml"
F1T_INST="$WS/install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml"
[ -e "$(dirname "$F1T_SRC")" ]  && ln -sf "$SCRIPT_DIR/config/vesc.yaml" "$F1T_SRC"
[ -e "$(dirname "$F1T_INST")" ] && ln -sf "$SCRIPT_DIR/config/vesc.yaml" "$F1T_INST"

echo "[3/4] colcon build"
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build \
    --packages-select race_stack q2_gap_follow roboracer_1v1 \
    --symlink-install

echo "[4/4] udev rule 설치 (VESC 자동 심링크)"
sudo cp "$SCRIPT_DIR/systemd/99-vesc.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger || true

echo ""
echo "======================================"
echo "  설치 완료!"
echo "======================================"
echo ""
echo "  [맵핑 → 웨이포인트 → 자율주행]"
echo "  1) ./race.sh mapping          ← SLAM 맵핑 (키보드로 트랙 주행)"
echo "  2) ./race.sh savemap <이름>   ← 맵 저장"
echo "  3) ./race.sh waypoints <이름> ← 레이싱라인 생성 (디스플레이 필요)"
echo "  4) race_script.yaml 수정"
echo "     map_name: <이름>"
echo "     waypoints: <이름>_wp"
echo "  5) ./race.sh q1               ← 자율주행"
echo ""
echo "  [수동 조종]"
echo "  ./race.sh joy                 ← 조이스틱 (LB=모드토글)"
echo "  ./race.sh teleop              ← 키보드"
