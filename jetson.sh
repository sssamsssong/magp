#!/usr/bin/env bash
set -euo pipefail

JETSON_HOST="${JETSON_HOST:-jetson@192.168.45.246}"
JETSON_WS="${JETSON_WS:-/home/jetson/f1tenth_ws}"
SESSION="${ROBORACER_SESSION:-roboracer}"
CMD="${1:-status}"

case "$CMD" in
  q1|q2|q3|final|mapping)
    ssh -t "$JETSON_HOST" "tmux new-session -d -s $SESSION -c $JETSON_WS './race.sh $CMD; bash' 2>/dev/null || tmux new-window -t $SESSION -c $JETSON_WS './race.sh $CMD; bash'; tmux attach -t $SESSION"
    ;;
  savemap)
    NAME="${2:?usage: ./jetson.sh savemap <map_name>}"
    ssh -t "$JETSON_HOST" "cd $JETSON_WS && ./race.sh savemap '$NAME'"
    ;;
  log)
    NAME="${2:?usage: ./jetson.sh log <waypoint_name>}"
    ssh -t "$JETSON_HOST" "tmux new-session -d -s $SESSION -c $JETSON_WS './race.sh log $NAME; bash' 2>/dev/null || tmux new-window -t $SESSION -c $JETSON_WS './race.sh log $NAME; bash'; tmux attach -t $SESSION"
    ;;
  stop)
    ssh -t "$JETSON_HOST" "cd $JETSON_WS && ./race.sh stop; tmux kill-session -t $SESSION 2>/dev/null || true"
    ;;
  status)
    ssh -t "$JETSON_HOST" "tmux ls 2>/dev/null || true; source /opt/ros/humble/setup.bash; source $JETSON_WS/install/setup.bash 2>/dev/null || true; ros2 node list 2>/dev/null || true"
    ;;
  attach)
    ssh -t "$JETSON_HOST" "tmux attach -t $SESSION"
    ;;
  *)
    echo "usage: ./jetson.sh q1|q2|q3|final|mapping|savemap <name>|log <name>|stop|status|attach"
    exit 2
    ;;
esac
