#!/usr/bin/env bash
# ============================================================
#  race.sh — 딸깍 스크립트
#
#  사용법:
#    ./race.sh                 race_script.yaml의 mode대로 실행
#    ./race.sh mapping         모드 임시 오버라이드 (yaml 안 바꿔도 됨)
#    ./race.sh mapping-joy     조이스틱으로 SLAM 맵핑
#    ./race.sh q1|q2|q3|race
#    ./race.sh savemap <이름>  맵핑 끝난 뒤 맵 저장
#    ./race.sh teleop          키보드 텔레옵만 단독 실행
#    ./race.sh joy             조이스틱 수동 조종 (브링업 포함)
#    ./race.sh log <이름>      웨이포인트 기록 (수동주행하며 레이싱라인 녹화)
#    ./race.sh waypoints [맵]  SLAM 맵에서 레이싱라인 생성 (GUI, 디스플레이 필요)
# ============================================================
set -e

WS=${RACE_WS:-$HOME/f1tenth_ws}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RACE_SCRIPT="$SCRIPT_DIR/race_script.yaml"
PKG_SHARE_MAPS="$WS/src/race_stack/maps"
PKG_SHARE_WP="$WS/src/race_stack/waypoints"

# ---------- 0. 환경 소싱 ----------
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# ---------- 1. VESC 심링크 (재부팅마다 필요) ----------
if [ ! -e /dev/sensors/vesc ]; then
    echo "[race.sh] /dev/sensors/vesc 심링크 생성"
    sudo mkdir -p /dev/sensors
    sudo ln -sf /dev/ttyACM0 /dev/sensors/vesc
fi

# ---------- 1-1. loopback 멀티캐스트 (재부팅마다 필요) ----------
# 이 젯슨은 lo 에 MULTICAST 플래그가 없어서, 같은 호스트 두 프로세스도
# 오직 WiFi/이더넷 멀티캐스트로만 서로를 발견한다. WiFi만 연결된 상태(SSH)
# 에서는 discovery 가 불안정해져 다른 터미널의 /map(slam_toolbox)을 못 찾아
# savemap 이 실패한다. lo 멀티캐스트를 켜서 loopback discovery 경로를 열어준다.
if ! ip link show lo | head -1 | grep -q "MULTICAST"; then
    echo "[race.sh] lo 멀티캐스트 활성화 (같은 호스트 DDS discovery 안정화)"
    sudo ip link set dev lo multicast on
fi

# ---------- 2. 모드 결정 ----------
export RACE_SCRIPT   # launch 파일이 이 env로 yaml 경로를 읽음
CMD=${1:-}
get_yaml () { python3 -c "
import yaml,sys
d=yaml.safe_load(open('$RACE_SCRIPT'))
k='$1'.split('.'); v=d
for kk in k: v=v[kk]
print(v)"; }

case "$CMD" in
  savemap)
    NAME=${2:?맵 이름을 주세요: ./race.sh savemap my_track}

    # /map 발행자가 없으면 map_saver 는 그냥 타임아웃만 내고 끝난다.
    # 원인을 알려주는 게 낫다 — 대부분 맵핑 창을 Ctrl+C 로 먼저 꺼버린 경우다.
    if ! ros2 topic info /map 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        echo "[race.sh] ✗ /map 을 발행하는 노드가 없습니다 (slam_toolbox 미실행)."
        echo ""
        echo "  맵은 slam_toolbox 가 메모리에 들고 있습니다."
        echo "  맵핑 창을 Ctrl+C 로 끄면 맵도 함께 사라져서 저장할 수 없습니다."
        echo ""
        echo "  올바른 순서:"
        echo "    1) 터미널A:  ./race.sh mapping-joy    ← 켜 둔 채로"
        echo "    2) 주행으로 트랙 한 바퀴"
        echo "    3) 터미널B:  ./race.sh savemap $NAME  ← A를 끄기 전에!"
        echo "    4) 저장 확인 후 터미널A 에서 Ctrl+C"
        exit 1
    fi

    # slam 은 odom TF 가 있어야 맵을 만든다. vesc_to_odom 은 서보 명령을 한 번
    # 받기 전까지 odom 을 발행하지 않으므로, 주행을 전혀 안 했으면 맵이 비어 있다.
    if ! ros2 topic info /odom 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        echo "[race.sh] ⚠ /odom 발행자가 없습니다 — 맵이 비어 있을 수 있습니다."
        echo "          조종기로 조금이라도 주행한 뒤 저장하세요."
    fi

    echo "[race.sh] 맵 저장 → $PKG_SHARE_MAPS/$NAME"
    ros2 run nav2_map_server map_saver_cli -f "$PKG_SHARE_MAPS/$NAME" --ros-args -p map_subscribe_transient_local:=true
    # install 쪽에도 복사 (rebuild 없이 바로 사용)
    INSTALL_MAPS="$WS/install/race_stack/share/race_stack/maps"
    [ -d "$INSTALL_MAPS" ] && cp "$PKG_SHARE_MAPS/$NAME".* "$INSTALL_MAPS/" || true
    echo "[race.sh] 완료. race_script.yaml의 map_name을 '$NAME'으로 바꾸세요."
    exit 0 ;;
  teleop)
    exec ros2 run race_stack keyboard_teleop --ros-args -p max_speed:=$(get_yaml speed.mapping_max) ;;
  joy)
    exec ros2 launch race_stack joystick.launch.py ;;
  log)
    NAME=${2:?웨이포인트 이름을 주세요: ./race.sh log my_track_wp}
    exec ros2 run race_stack waypoint_logger --ros-args -p output:="$PKG_SHARE_WP/$NAME.csv" ;;
  waypoints)
    NAME=${2:-}
    if [ -n "$NAME" ]; then
      exec ros2 run race_stack gen_waypoints -- --map "$NAME"
    else
      exec ros2 run race_stack gen_waypoints
    fi ;;
  "" ) MODE=$(get_yaml mode) ;;
  mapping|mapping-joy|q1|q2|q3|race) MODE=$CMD ;;
  * ) echo "알 수 없는 명령: $CMD"; exit 1 ;;
esac

echo "============================================"
echo "  RoboRacer 2026  |  MODE = $MODE"
echo "  config: $RACE_SCRIPT"
echo "============================================"

if [ "$MODE" = "mapping" ]; then
    # SLAM 맵핑: 브링업+slam_toolbox는 백그라운드, 키보드 텔레옵은 포그라운드(tty 필요)
    ros2 launch race_stack mapping.launch.py &
    LAUNCH_PID=$!
    sleep 5
    echo ""
    echo ">>> 키보드로 천천히 트랙 1~2바퀴 돌면서 맵을 만드세요."
    echo ">>> ★ 이 창을 끄지 말고, 다른 터미널에서 저장하세요:"
    echo ">>>       ./race.sh savemap <맵이름>"
    echo ">>> ★ 저장이 끝난 뒤에 이 창에서 Ctrl+C  (먼저 끄면 /map이 사라져 저장 실패)"
    echo ""
    trap "kill $LAUNCH_PID 2>/dev/null" EXIT
    ros2 run race_stack keyboard_teleop --ros-args -p max_speed:=$(get_yaml speed.mapping_max)

elif [ "$MODE" = "mapping-joy" ]; then
    # SLAM 맵핑 + 조이스틱: 브링업+slam_toolbox 백그라운드, joy는 별도 실행
    ros2 launch race_stack mapping.launch.py &
    LAUNCH_PID=$!
    sleep 5
    echo ""
    echo ">>> 조이스틱으로 천천히 트랙 1~2바퀴 돌면서 맵을 만드세요."
    echo ">>> ★ 이 창을 끄지 말고, 다른 터미널에서 저장하세요:"
    echo ">>>       ./race.sh savemap <맵이름>"
    echo ">>> ★ 저장이 끝난 뒤에 이 창에서 Ctrl+C  (먼저 끄면 /map이 사라져 저장 실패)"
    echo ""

    JOY_DEV=$(get_yaml joystick.device 2>/dev/null || echo "/dev/input/js0")
    JOY_ID=$(echo "$JOY_DEV" | grep -o '[0-9]*$'); JOY_ID=${JOY_ID:-0}
    MAX_SPD=$(get_yaml speed.mapping_max)
    AX_STEER=$(get_yaml joystick.axis_steer)
    AX_SPEED=$(get_yaml joystick.axis_speed)
    BTN_REV=$(get_yaml joystick.btn_reverse)
    BTN_TOG=$(get_yaml joystick.btn_toggle)
    STEER_INV=$(get_yaml joystick.steer_invert 2>/dev/null || echo "False")
    TRIG_INV=$(get_yaml joystick.trigger_invert 2>/dev/null || echo "True")
    TRIG_DZ=$(get_yaml joystick.trigger_deadzone 2>/dev/null || echo "0.05")
    TRIG_EXPO=$(get_yaml joystick.trigger_expo 2>/dev/null || echo "1.0")
    STEER_DZ=$(get_yaml joystick.steer_deadzone 2>/dev/null || echo "0.08")
    ARM=$(get_yaml joystick.arm_on_neutral 2>/dev/null || echo "True")
    JOY_DBG=$(get_yaml joystick.debug 2>/dev/null || echo "False")

    # deadzone 0 → 트리거 선형성 보존 (데드존은 joystick_teleop이 처리)
    # autorepeat 20Hz → RT를 고정해도 mux(timeout 0.2s)가 안 끊김
    ros2 run joy joy_node --ros-args \
        -p device_id:="$JOY_ID" -p deadzone:=0.0 -p autorepeat_rate:=20.0 &
    JOY_PID=$!
    trap "kill $LAUNCH_PID $JOY_PID 2>/dev/null" EXIT

    ros2 run race_stack joystick_teleop --ros-args \
        -p max_speed:="$MAX_SPD" \
        -p axis_steer:="$AX_STEER" \
        -p axis_speed:="$AX_SPEED" \
        -p btn_reverse:="$BTN_REV" \
        -p btn_toggle:="$BTN_TOG" \
        -p steer_invert:="$STEER_INV" \
        -p trigger_invert:="$TRIG_INV" \
        -p trigger_deadzone:="$TRIG_DZ" \
        -p trigger_expo:="$TRIG_EXPO" \
        -p steer_deadzone:="$STEER_DZ" \
        -p arm_on_neutral:="$ARM" \
        -p debug:="$JOY_DBG"
else
    exec ros2 launch race_stack race.launch.py mode:=$MODE
fi
