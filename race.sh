#!/usr/bin/env bash
# ============================================================
#  race.sh — 딸깍 스크립트
#
#  사용법:
#    ./race.sh                 race_script.yaml의 mode대로 실행
#    ./race.sh mapping         모드 임시 오버라이드 (yaml 안 바꿔도 됨)
#    ./race.sh mapping-joy     조이스틱으로 SLAM 맵핑
#    ./race.sh q1|q2|q3|race [컨트롤러]
#                              컨트롤러 생략 시 yaml의 controller 값 사용
#                              (auto | stanley | pure_pursuit | gap_follower)
#                              예: ./race.sh q1 pure_pursuit
#    ./race.sh box [모드]      ★ 박스 위 벤치 테스트 — 바퀴를 돌리지 않는다.
#                              실주행과 동일한 스택(맵/AMCL/LiDAR/컨트롤러)을 띄우되
#                              속도만 0. 토픽 통신·RViz 표시·조향 서보 확인용.
#    ./race.sh check           토픽이 실제로 발행 중인지 표로 확인 (주행 중 별도 터미널)
#    ./race.sh stop            돌고 있는 스택 전부 정리
#    ./race.sh savemap <이름>  맵핑 끝난 뒤 맵 저장
#    ./race.sh teleop          키보드 텔레옵만 단독 실행
#    ./race.sh joy             조이스틱 수동 조종 (브링업 포함)
#    ./race.sh log <이름>      웨이포인트 기록 (수동주행하며 레이싱라인 녹화)
#    ./race.sh waypoints [맵]  SLAM 맵에서 레이싱라인 생성 (GUI, 디스플레이 필요)
#
#  게인 튜닝은 ./tune.sh — race_script.yaml 을 열지 않고 값만 바꾼다.
#    ./tune.sh                       현재 값 보기
#    ./tune.sh stanley.gain=1.2      설정 → 다음 ./race.sh 부터 적용 (재빌드 불필요)
# ============================================================
set -e

WS=${RACE_WS:-$HOME/f1tenth_ws}
# gen_waypoints 등 파이썬 노드는 RACE_WS 로 맵/웨이포인트 폴더를 찾는다. export 안 하면
# __file__ 기준으로 build/race_stack/maps(빌드 시점 스냅샷 복사본)를 읽어 새 맵이 안 보인다.
# WS/src/race_stack 는 repo(~/yax)로 심링크돼 있으므로 여기를 가리키게 export.
export RACE_WS="$WS"
# ~/f1tenth_ws/race.sh 는 이 repo로 가는 심링크 — readlink -f 로 실제 위치(~/yax)를
# 찾아야 race_script.yaml 을 제대로 읽는다 (심링크 경유 실행 시 launch가 즉사하던 버그)
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
RACE_SCRIPT="$SCRIPT_DIR/race_script.yaml"
PKG_SHARE_MAPS="$WS/src/race_stack/maps"
PKG_SHARE_WP="$WS/src/race_stack/waypoints"

# ---------- 0. 환경 소싱 ----------
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# DDS 설정 (race_script.yaml 의 network: 섹션). 노트북에서 RViz 로 관찰할 때
# 통신 안정성을 좌우한다 — rviz.sh 도 같은 파일을 source 해서 양쪽을 맞춘다.
export RACE_SCRIPT="$SCRIPT_DIR/race_script.yaml"
[ -f "$SCRIPT_DIR/config/dds_env.sh" ] && source "$SCRIPT_DIR/config/dds_env.sh"

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

# 돌고 있는 스택 프로세스 PID 목록.
# ★ launch 부모만 찾으면 안 된다. 터미널을 pkill 하거나 SSH 가 끊기면 부모(launch)만
#   죽고 urg_node·map_server·컨트롤러가 고아로 살아남는다. 이 상태를 탐지 못 하면
#   다음 실행에서 LiDAR TCP 포트·VESC 시리얼을 고아가 쥐고 있어 그대로 충돌한다.
#   그래서 부모와 자식 노드를 모두 본다.
#   ★ 빠짐없이 적을 것. joy_node/joy_teleop/static_transform_publisher 가 빠져 있어
#     실행할 때마다 누적됐고, 6세대 17개가 RTPS 포트(7400/7410~)를 쥔 채 남아 있었다.
#     FastDDS 는 견뎠지만 CycloneDDS 는 참여자 인덱스를 못 찾아 스택 전체가 죽었다
#     ("Failed to find a free participant index for domain 0").
STACK_PROCS="[r]ace\.launch\.py|[m]apping\.launch\.py|[u]rg_node_driver|[v]esc_driver_node|\
[n]av2_amcl/amcl|[n]av2_map_server/map_server|[l]ifecycle_manager|[s]tanley_controller|\
[m]ission_manager|[p]ure_pursuit|[g]ap_follower|[s]afety_node|[a]ckermann_mux|\
[a]ckermann_to_vesc_node|[v]esc_to_odom_node|[s]lam_toolbox|\
[j]oy_node|[j]oy_teleop|[s]tatic_transform_publisher|[k]eyboard_teleop|[j]oystick_teleop|\
[w]aypoint_logger"
running_pids () { pgrep -f "$STACK_PROCS" 2>/dev/null || true; }

kill_stack () {
    [ -z "$(running_pids)" ] && return 0
    # launch 부모부터 정리해야 자식이 respawn 되지 않는다.
    local parents; parents=$(pgrep -f "[r]ace\.launch\.py|[m]apping\.launch\.py" 2>/dev/null || true)
    if [ -n "$parents" ]; then
        echo "[race.sh] launch 종료 (PID: $(echo $parents | tr '\n' ' '))"
        kill $parents 2>/dev/null || true
        sleep 3
    fi
    local orphans; orphans=$(running_pids)
    if [ -n "$orphans" ]; then
        echo "[race.sh] 남은 노드 정리 (PID: $(echo $orphans | tr '\n' ' '))"
        kill $orphans 2>/dev/null || true
        sleep 2
        orphans=$(running_pids)
        [ -n "$orphans" ] && { kill -9 $orphans 2>/dev/null || true; sleep 1; }
    fi
    if [ -n "$(running_pids)" ]; then
        echo "[race.sh] ⚠ 아직 남은 프로세스가 있습니다. 수동 확인:"
        echo "          ps -ef | grep -E 'race.launch|urg_node|vesc_driver|map_server'"
        return 1
    fi
    echo "[race.sh] 정리 완료"
}

case "$CMD" in
  stop)
    kill_stack; exit $? ;;
  check)
    # 주행 중 다른 터미널에서 실행 — 각 토픽이 실제로 발행되고 있는지 확인.
    # ros2 topic list 는 discovery 만 되면 뜨므로 판정 근거가 못 된다.
    # Publisher count 와 실제 수신 여부를 둘 다 본다.
    echo "============================================"
    echo "  토픽 상태 점검  |  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    echo "============================================"
    printf "%-20s %-10s %s\n" "TOPIC" "PUB" "수신"
    printf "%-20s %-10s %s\n" "--------------------" "----------" "------"
    for t in /map /scan /odom /tf /amcl_pose /drive /ackermann_cmd \
             /commands/motor/speed /race/enabled /waypoints_path; do
        n=$(ros2 topic info "$t" 2>/dev/null | sed -n 's/Publisher count: //p')
        n=${n:-0}
        if [ "$n" -gt 0 ] 2>/dev/null; then
            # transient_local(/map, /race/enabled)도 잡히도록 echo 로 실수신 확인.
            # 젯슨에서 ros2 CLI 노드 기동이 느려 5초로는 40Hz 토픽도 오탐이 난다.
            if timeout 10 ros2 topic echo "$t" --once >/dev/null 2>&1; then
                rx="OK"
            else
                rx="✗ 발행자는 있는데 데이터 없음"
            fi
        else
            rx="✗ 발행자 없음"
        fi
        printf "%-20s %-10s %s\n" "$t" "$n" "$rx"
    done
    echo ""
    echo "  ※ 노트북에서 이 스크립트를 돌리면 젯슨↔노트북 DDS 연결까지 검증됩니다."
    exit 0 ;;
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
  "" ) MODE=$(get_yaml mode); CTRL="" ;;
  mapping|mapping-joy) MODE=$CMD; CTRL="" ;;
  box)
    # 박스 위 벤치 테스트. 모드는 인자 > yaml 순으로 결정 — 실주행과 같은 스택을
    # 그대로 띄워야 검증이 의미가 있다. 차이는 bench:=true (속도 0, 자동정지 없음)뿐.
    MODE=${2:-$(get_yaml mode)}
    CTRL=""
    BENCH=1 ;;
  q1|q2|q3|race)
    MODE=$CMD
    CTRL=${2:-}   # 두 번째 인자로 컨트롤러 오버라이드 (yaml 수정 불필요)
    case "$CTRL" in
      ""|auto|stanley|pure_pursuit|gap_follower) ;;
      * ) echo "알 수 없는 컨트롤러: $CTRL  (auto|stanley|pure_pursuit|gap_follower)"; exit 1 ;;
    esac ;;
  * ) echo "알 수 없는 명령: $CMD"; exit 1 ;;
esac

# ---------- 2-1. 중복 실행 가드 ----------
# 스택이 두 벌 뜨면 LiDAR TCP 포트(1클라이언트)·VESC 시리얼을 서로 뺏어
# urg_node 연결 실패, vesc_driver 세그폴트, 같은 이름 노드 충돌로 map_server
# lifecycle 활성화 실패가 연쇄로 터진다. 증상이 하드웨어 고장처럼 보여 디버깅이 샌다.
if [ -n "$(running_pids)" ]; then
    echo "[race.sh] ⚠ 이미 실행 중인 스택이 있습니다 (PID: $(running_pids | tr '\n' ' '))"
    echo "          두 벌이 뜨면 LiDAR/VESC 를 서로 뺏어 전부 실패합니다."
    read -r -p "          기존 스택을 정리하고 계속할까요? [Y/n] " ans
    case "$ans" in
        [nN]*) echo "[race.sh] 중단했습니다."; exit 1 ;;
        *) kill_stack || exit 1 ;;
    esac
fi

echo "============================================"
echo "  RoboRacer 2026  |  MODE = $MODE${CTRL:+  |  CONTROLLER = $CTRL}"
echo "  config: $RACE_SCRIPT"
if [ -n "${BENCH:-}" ]; then
echo "  ★ BENCH (박스 위) — 속도 0, 바퀴 안 돎, 자동정지 없음"
echo "     확인용: 다른 터미널에서  ./race.sh check   /  노트북에서  ./rviz.sh"
fi
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
    exec ros2 launch race_stack race.launch.py mode:=$MODE \
        ${CTRL:+controller:=$CTRL} ${BENCH:+bench:=true}
fi
