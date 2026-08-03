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
#    - 노트북과 젯슨이 같은 네트워크(192.168.50.x)
#    - 양쪽 ROS_DOMAIN_ID 동일 (기본 미설정=0). 노트북 ~/.bashrc 에서
#      ROS_DOMAIN_ID 를 바꿔놨다면 젯슨과 같은 값으로 맞출 것.
#
#  사용:  ./rviz.sh
#
#  ▶ 젯슨에서 직접 띄우기 (모니터 연결 시) — 초기 위치 정렬에 편하다
#      SSH 로 들어와서 그냥 ./rviz.sh  (DISPLAY 는 자동으로 잡는다)
#      터미널1:  ./race.sh box       스택 기동 (바퀴 안 돌아감)
#      터미널2:  ./rviz.sh           젯슨 화면에 RViz → 2D Pose Estimate 로 정렬
#      터미널3:  ./race.sh savepose  맞춘 위치 저장
#      ※ 주행 중 관찰용이면 노트북 로컬 실행이 낫다 (젯슨 CPU 를 안 뺏김)
# ============================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$DIR/config/race.rviz"

source /opt/ros/humble/setup.bash

# DDS 설정 — 젯슨의 race.sh 와 같은 파일을 읽어 양쪽 미들웨어를 일치시킨다.
# (RMW 가 한쪽만 다르면 토픽이 하나도 안 보인다)
export RACE_SCRIPT="$DIR/race_script.yaml"
[ -f "$DIR/config/dds_env.sh" ] && source "$DIR/config/dds_env.sh"

# SSH 세션이면 RViz 창은 이 명령을 받은 기기(젯슨) 화면에 뜬다.
# 둘 다 정상적인 쓰임새라 막지 않고, 어느 쪽인지만 알려준다:
#   - 노트북 로컬 실행  → 노트북 화면에서 관찰 (주행 중 권장. 젯슨 CPU 안 씀)
#   - 젯슨에 SSH + 모니터 → 젯슨 화면에 뜸. 2D Pose Estimate 후 곧바로
#                           ./race.sh savepose 할 수 있어 초기 정렬에 편하다.
if [ -n "${SSH_CONNECTION:-}" ]; then
    # SSH 세션은 보통 DISPLAY 가 비어 있다. 로컬 X 세션이 있으면 그걸 잡아준다.
    if [ -z "${DISPLAY:-}" ]; then
        for sock in /tmp/.X11-unix/X*; do
            [ -e "$sock" ] || continue
            export DISPLAY=":${sock##*/X}"
            break
        done
    fi
    if [ -z "${DISPLAY:-}" ]; then
        echo "✗ 젯슨에 그래픽 세션이 없습니다 (DISPLAY 미설정, /tmp/.X11-unix 비어 있음)."
        echo "  젯슨에 모니터를 연결했는지 확인하거나, 노트북 로컬에서 ./rviz.sh 를 쓰세요."
        exit 1
    fi
    echo "ℹ SSH 세션 — RViz 창은 젯슨 화면(DISPLAY=$DISPLAY)에 뜹니다."
    echo "  초기 정렬 흐름:  RViz 에서 2D Pose Estimate  →  다른 터미널에서 ./race.sh savepose"
    echo "  ※ 주행 중에는 젯슨 CPU 를 나눠 쓰므로, 관찰용이면 노트북 로컬 실행을 권장합니다."
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
