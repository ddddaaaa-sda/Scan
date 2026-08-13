#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${SCAN_RUN_DIR:-/tmp/scan_upper_stack}"

set +u
source /opt/ros/humble/setup.bash
set -u

if command -v ros2 >/dev/null 2>&1; then
  timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
fi

for name in follower scan_planner fast_lio livox; do
  pid_file="$RUN_DIR/$name.pid"
  if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
    echo "Stopping $name, pid=$(cat "$pid_file")"
    kill "$(cat "$pid_file")" 2>/dev/null || true
  fi
done

if pgrep -f "rviz2" >/dev/null 2>&1; then
  echo "Stopping rviz2 ..."
  pkill -f "rviz2" 2>/dev/null || true
fi

sleep 2

for name in follower scan_planner fast_lio livox; do
  pid_file="$RUN_DIR/$name.pid"
  if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
    echo "Force stopping $name, pid=$(cat "$pid_file")"
    kill -9 "$(cat "$pid_file")" 2>/dev/null || true
  fi
  rm -f "$pid_file"
done

if pgrep -f "rviz2" >/dev/null 2>&1; then
  echo "Force stopping rviz2 ..."
  pkill -9 -f "rviz2" 2>/dev/null || true
fi

echo "Upper stack stopped."
