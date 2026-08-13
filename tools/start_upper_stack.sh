#!/usr/bin/env bash
set -euo pipefail

# Start Livox, FAST-LIO, SCAN-Planner, and cmd_vel follower on the upper board.
# Logs and PIDs are stored in /tmp/scan_upper_stack.
#
# Usage:
#   bash tools/start_upper_stack.sh
#   bash tools/start_upper_stack.sh --configure-link

RUN_DIR="${SCAN_RUN_DIR:-/tmp/scan_upper_stack}"
LIVOX_WS="${LIVOX_WS:-$HOME/livox_ws}"
LIO_WS="${LIO_WS:-$HOME/lio_ws}"
SCAN_WS="${SCAN_WS:-$HOME/scan_ws}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "$RUN_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$SCAN_WS/install/setup.bash"
set -u

if [[ "${1:-}" == "--configure-link" ]]; then
  echo "Configuring enP8p1s0 as 192.168.1.5/24 ..."
  sudo ip addr flush dev enP8p1s0
  sudo ip addr add 192.168.1.5/24 dev enP8p1s0
  sudo ip link set enP8p1s0 up
fi

start_bg() {
  local name="$1"
  local cmd="$2"
  local log="$RUN_DIR/$name.log"
  local pid_file="$RUN_DIR/$name.pid"

  if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
    echo "$name already running, pid=$(cat "$pid_file")"
    return
  fi

  echo "Starting $name ..."
  bash -lc "$cmd" >"$log" 2>&1 &
  echo "$!" >"$pid_file"
  echo "  pid=$(cat "$pid_file"), log=$log"
}

wait_for_topic() {
  local topic="$1"
  local timeout="${2:-30}"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "Detected $topic"
      return 0
    fi
    sleep 1
  done
  echo "Warning: timeout waiting for $topic; continuing."
  return 1
}

COMMON_ENV="set +u; source /opt/ros/humble/setup.bash; set -u; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

start_bg livox "$COMMON_ENV; cd '$LIVOX_WS'; set +u; source install/setup.bash; set -u; exec ros2 launch livox_ros_driver2 msg_MID360_launch.py"
wait_for_topic "/livox/lidar" 30 || true

start_bg fast_lio "$COMMON_ENV; cd '$LIO_WS'; set +u; source '$LIVOX_WS/install/setup.bash'; source install/setup.bash; set -u; exec ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz_cfg:='$SCAN_WS/config/fastlio_scan.rviz'"
wait_for_topic "/Odometry" 45 || true
wait_for_topic "/cloud_registered" 45 || true

start_bg scan_planner "$COMMON_ENV; cd '$SCAN_WS'; set +u; source install/setup.bash; set -u; exec ros2 launch scan_planner scan_planner.launch.py use_rviz:=false cloud_topic:=/cloud_registered odom_topic:=/Odometry global_path_topic:=/scan_global_path planning_frame:=camera_init"
sleep 3

start_bg follower "$COMMON_ENV; cd '$SCAN_WS'; set +u; source install/setup.bash; set -u; exec ros2 launch scan_cmd_vel_follower scan_cmd_vel_follower.launch.py path_topic:=/visual_local_trajectory odom_topic:=/Odometry cmd_topic:=/cmd_vel planning_frame:=camera_init max_linear_x:=0.3 max_angular_z:=0.2 odom_timeout:=2.0 path_timeout:=2.0"

echo
echo "Upper stack started."
echo "Logs: tail -f $RUN_DIR/*.log"
echo "Trigger planning: bash $SCAN_WS/tools/trigger_scan_plan.sh start"
echo "Stop planning:   bash $SCAN_WS/tools/trigger_scan_plan.sh stop"
echo "Stop all:         bash $SCAN_WS/tools/stop_upper_stack.sh
"
echo
echo "Important: do not immediately send trigger false during real test."
