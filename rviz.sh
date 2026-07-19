#!/usr/bin/env bash
# ============================================================
#  rviz.sh — ★ 노트북에서 실행 ★  젯슨 주행을 실시간 RViz로 관찰
#
#  젯슨에 SSH로 들어가 ./race.sh 를 돌리는 것과 별개로,
#  노트북 로컬 터미널(SSH 아님)에서 이 스크립트를 실행하면
#  맵 + 레이싱라인 + 차 위치/궤적 + LiDAR + 목표점이 뜬다.
#
#  전제:
#    - 노트북에 ROS 2 Humble + rviz2 설치
#      (sudo apt install ros-humble-desktop  또는  ros-humble-rviz2)
#    - 노트북과 젯슨이 같은 네트워크(192.168.45.x)
#    - 양쪽 ROS_DOMAIN_ID 동일 (기본 미설정=0). 노트북 ~/.bashrc 에서
#      ROS_DOMAIN_ID 를 바꿔놨다면 젯슨과 같은 값으로 맞출 것.
#
#  사용:  ./rviz.sh
# ============================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$DIR/config/race.rviz"

source /opt/ros/humble/setup.bash

# DDS 설정 — 젯슨의 race.sh 와 같은 파일을 읽어 양쪽 미들웨어를 일치시킨다.
# (RMW 가 한쪽만 다르면 토픽이 하나도 안 보인다)
export RACE_SCRIPT="$DIR/race_script.yaml"
[ -f "$DIR/config/dds_env.sh" ] && source "$DIR/config/dds_env.sh"

# SSH 세션이면 RViz 창이 젯슨 모니터에 뜬다 — 노트북에서 보려는 의도와 어긋난다.
# rviz.sh 는 "노트북에서 네이티브 실행"이 전제다 (화면 전송이 아니라 토픽 구독).
if [ -n "${SSH_CONNECTION:-}" ]; then
    echo "⚠ SSH 세션에서 실행 중입니다."
    echo "  RViz 창은 이 명령을 받은 기기(젯슨)의 화면에 뜹니다."
    echo "  노트북에서 보려면 SSH 를 끊고 노트북 로컬 터미널에서 실행하세요."
    echo "  (rviz.sh 는 화면을 전송하는 게 아니라, 노트북이 직접 토픽을 구독해 그립니다)"
    echo ""
fi

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "✗ rviz2 가 없습니다.  sudo apt install ros-humble-rviz2"
    exit 1
fi
[ -f "$CFG" ] || { echo "✗ RViz 설정 없음: $CFG (repo 최신화 필요: git pull)"; exit 1; }

echo "============================================"
echo "  RViz 실시간 관찰  |  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "  config: $CFG"
echo "  ※ 젯슨에서 ./race.sh 주행이 돌고 있어야 토픽이 보입니다."
echo "============================================"
# 젯슨 토픽이 잡히는지 5초만 살짝 확인 (안 잡혀도 RViz 는 그냥 실행)
if ! ros2 topic list 2>/dev/null | grep -q "/waypoints_path"; then
    echo "  ⚠ 아직 /waypoints_path 가 안 보입니다."
    echo "    - 젯슨에서 주행(race.sh q1/q2/q3/race)이 실행 중인지"
    echo "    - 노트북·젯슨 네트워크/ROS_DOMAIN_ID 가 같은지 확인하세요."
    echo "    (그래도 RViz 는 띄웁니다 — 주행 시작하면 자동으로 나타남)"
fi

exec rviz2 -d "$CFG"
