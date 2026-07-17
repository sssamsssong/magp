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
