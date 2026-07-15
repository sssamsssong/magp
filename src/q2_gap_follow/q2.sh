#!/bin/bash
# =============================================================================
# q2.sh — Q2 시뮬레이션 통합 실행 스크립트
# 기존 sim.sh 옆에 두고 사용. 컨테이너 이름은 sim.sh 와 동일하게 맞춤.
#
# 사용:
#   ./q2.sh setup           # q2_gap_follow 패키지를 컨테이너에 마운트되도록 준비
#   ./q2.sh build           # 컨테이너 내에서 colcon build
#   ./q2.sh map [N] [seed]  # levine 맵에 장애물 N개 박은 levine_q2 생성 (기본 N=2)
#   ./q2.sh sim             # 시뮬레이터 실행 (levine_q2 사용)
#   ./q2.sh run             # q2_driver 실행
#   ./q2.sh logs            # q2_driver 로그 follow
#   ./q2.sh topic           # /drive /scan /q2/mode 토픽 hz
# =============================================================================

set -e

# 사용자 환경에 맞게 수정
SIM_CONTAINER="f1tenth_gym_ros-main-sim-1"
SIM_DIR_HOST="${SIM_DIR_HOST:-$(cd "$(dirname "$0")/.." && pwd)/f1tenth_gym_ros-main}"
Q2_PKG_HOST="${Q2_PKG_HOST:-$(cd "$(dirname "$0")" && pwd)}"

ensure_container() {
    if ! docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$"; then
        echo "[q2] 컨테이너가 떠 있지 않음. 먼저 ./sim.sh up 실행하세요."
        exit 1
    fi
}

cmd_setup() {
    echo "[q2] q2_gap_follow 패키지를 컨테이너에 복사..."
    ensure_container
    # docker-compose에 볼륨을 추가하지 않고도 일단 cp로 넣어버림
    docker cp "$Q2_PKG_HOST" "$SIM_CONTAINER:/sim_ws/src/"
    echo "[q2] 완료. 다음: ./q2.sh build"
}

cmd_build() {
    ensure_container
    echo "[q2] colcon build (q2_gap_follow)..."
    docker exec -it "$SIM_CONTAINER" bash -c "
        source /opt/ros/foxy/setup.bash &&
        cd /sim_ws &&
        colcon build --packages-select q2_gap_follow --symlink-install
    "
    echo "[q2] 빌드 완료"
}

cmd_map() {
    ensure_container
    N="${1:-2}"
    SEED="${2:-$RANDOM}"
    echo "[q2] levine 맵에 장애물 ${N}개 추가 (seed=$SEED)..."
    docker exec -it "$SIM_CONTAINER" bash -c "
        cd /sim_ws/src/f1tenth_gym_ros/maps &&
        python3 /sim_ws/src/q2_gap_follow/scripts/add_obstacles_to_map.py \
            --map levine --out levine_q2 --n $N --seed $SEED --clearance 0.6
    "
    # sim.yaml의 맵 경로를 levine_q2로 교체
    docker exec "$SIM_CONTAINER" bash -c "
        sed -i \"s#map_path: .*#map_path: '/sim_ws/src/f1tenth_gym_ros/maps/levine_q2'#\" \
            /sim_ws/src/f1tenth_gym_ros/config/sim.yaml &&
        cp /sim_ws/src/f1tenth_gym_ros/config/sim.yaml \
           /sim_ws/install/f1tenth_gym_ros/share/f1tenth_gym_ros/config/sim.yaml
    "
    echo "[q2] 맵 적용 완료. 시뮬 재실행 필요."
}

cmd_sim() {
    ensure_container
    echo "[q2] 시뮬레이터 실행..."
    docker exec -it "$SIM_CONTAINER" bash -c "
        source /opt/ros/foxy/setup.bash &&
        source /sim_ws/install/setup.bash &&
        ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    "
}

cmd_run() {
    ensure_container
    echo "[q2] q2_driver 실행..."
    docker exec -it "$SIM_CONTAINER" bash -c "
        source /opt/ros/foxy/setup.bash &&
        source /sim_ws/install/setup.bash &&
        ros2 launch q2_gap_follow q2_launch.py
    "
}

cmd_logs() {
    ensure_container
    docker exec -it "$SIM_CONTAINER" bash -c "
        source /opt/ros/foxy/setup.bash &&
        source /sim_ws/install/setup.bash &&
        ros2 topic echo /q2/mode
    "
}

cmd_topic() {
    ensure_container
    docker exec -it "$SIM_CONTAINER" bash -c "
        source /opt/ros/foxy/setup.bash &&
        source /sim_ws/install/setup.bash &&
        echo '--- /scan ---' &&
        ros2 topic hz /scan --window 50 &
        sleep 3 && kill %1 2>/dev/null
        echo '--- /drive ---' &&
        ros2 topic hz /drive --window 50 &
        sleep 3 && kill %1 2>/dev/null
    "
}

cmd_help() {
    sed -n '3,17p' "$0"
}

case "${1:-help}" in
    setup)  cmd_setup ;;
    build)  cmd_build ;;
    map)    shift; cmd_map "$@" ;;
    sim)    cmd_sim ;;
    run)    cmd_run ;;
    logs)   cmd_logs ;;
    topic)  cmd_topic ;;
    *)      cmd_help ;;
esac
