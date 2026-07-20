#!/usr/bin/env bash
# ============================================================
#  sim.sh — 노트북에서 시뮬레이션 실행
#
#  최초 1회:  ./setup_laptop.sh
#  이후 실행: ./sim.sh
# ============================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

WS=${RACE_WS:-$HOME/f1tenth_ws}
export RACE_WS="$WS"
export RACE_SCRIPT="$SCRIPT_DIR/race_script.yaml"

SIM_WS="$SCRIPT_DIR/f1TENTH-contest_full_stack/sim_ws"

# ---------- 빌드 여부 확인 ----------
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "✗ ~/f1tenth_ws 가 빌드되지 않았습니다."
    echo "  먼저 ./setup_laptop.sh 를 실행하세요."
    exit 1
fi
if [ ! -f "$SIM_WS/install/setup.bash" ]; then
    echo "✗ sim_ws 가 빌드되지 않았습니다."
    echo "  먼저 ./setup_laptop.sh 를 실행하세요."
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$SIM_WS/install/setup.bash"

MAP=$(python3 -c "import yaml; print(yaml.safe_load(open('$RACE_SCRIPT'))['map_name'])" 2>/dev/null || echo "?")
MODE=$(python3 -c "import yaml; print(yaml.safe_load(open('$RACE_SCRIPT'))['mode'])" 2>/dev/null || echo "?")
CTRL=$(python3 -c "import yaml; print(yaml.safe_load(open('$RACE_SCRIPT'))['controller'])" 2>/dev/null || echo "?")

echo "============================================"
echo "  RoboRacer 2026  |  SIMULATION"
echo "  mode=$MODE  controller=$CTRL  map=$MAP"
echo "  config: $RACE_SCRIPT"
echo "============================================"

exec ros2 launch race_stack sim_race.launch.py
