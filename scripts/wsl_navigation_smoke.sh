#!/usr/bin/env bash
# End-to-end saved-map check: SLAM scan, save, restart, AMCL, navigate out and back.
source /opt/ros/humble/setup.bash
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/ros2/install/setup.bash"
set -euo pipefail
LOG_DIR="${TMPDIR:-/tmp}/ros2_navigation_smoke"
MAP_PREFIX="$LOG_DIR/smoke_map"
mkdir -p "$LOG_DIR"
rm -f "${MAP_PREFIX}.yaml" "${MAP_PREFIX}.pgm"

SIM_PID=""
NAV_PID=""
CONTROL_PID=""
stop_group() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-$pid" 2>/dev/null || break
    sleep 0.25
  done
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || true
    for _ in $(seq 1 12); do
      kill -0 -- "-$pid" 2>/dev/null || break
      sleep 0.25
    done
  fi
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
cleanup() {
  stop_group "$CONTROL_PID"
  stop_group "$NAV_PID"
  stop_group "$SIM_PID"
}
trap cleanup EXIT INT TERM

wait_topic() {
  python3 "$REPO_ROOT/scripts/wait_topic.py" "$1" "$2" --timeout "$3" "${@:4}"
}
start_sim() {
  setsid ros2 launch robot_bringup sim.launch.py gui:=false rviz:=false \
    > "$LOG_DIR/sim.log" 2>&1 &
  SIM_PID=$!
  wait_topic /clock rosgraph_msgs/msg/Clock 60 --sensor-qos
  wait_topic /odom nav_msgs/msg/Odometry 60
  wait_topic /scan sensor_msgs/msg/LaserScan 60 --sensor-qos
}

echo "[1/5] start simulator and SLAM mapping"
start_sim
setsid ros2 launch robot_bringup navigation.launch.py mode:=mapping \
  > "$LOG_DIR/mapping.log" 2>&1 &
NAV_PID=$!
wait_topic /map nav_msgs/msg/OccupancyGrid 60

echo "[2/5] rotate once to observe the environment"
setsid ros2 run robot_control robot_control_node > "$LOG_DIR/control.log" 2>&1 &
CONTROL_PID=$!
sleep 2
ros2 topic pub --once /robot_command robot_interfaces/msg/RobotCommand \
  "{action: rotate, angular_z: 0.4, duration: 16.0}" >/dev/null
sleep 17
ros2 topic pub --once /robot_command robot_interfaces/msg/RobotCommand \
  "{action: stop}" >/dev/null

echo "[3/5] save map and stop the first simulation"
wait_topic /map nav_msgs/msg/OccupancyGrid 15
saved_map=false
for attempt in 1 2 3; do
  if bash "$REPO_ROOT/scripts/wsl_save_map.sh" "$MAP_PREFIX"; then
    saved_map=true
    break
  fi
  echo "Map save attempt $attempt failed; retrying while SLAM remains active"
  sleep 1
done
if [ "$saved_map" != true ]; then
  echo "FAIL: could not save the live SLAM map"
  tail -60 "$LOG_DIR/mapping.log"
  exit 1
fi
stop_group "$CONTROL_PID"
CONTROL_PID=""
stop_group "$NAV_PID"
NAV_PID=""
stop_group "$SIM_PID"
SIM_PID=""

echo "[4/5] restart simulator with saved map and verify AMCL"
start_sim
setsid ros2 launch robot_bringup navigation.launch.py mode:=localization \
  "map:=${MAP_PREFIX}.yaml" > "$LOG_DIR/localization.log" 2>&1 &
NAV_PID=$!

echo "[5/5] navigate to a nearby goal and return to the origin"
python3 "$REPO_ROOT/scripts/navigation_smoke_client.py" round-trip \
  0.8 0.0 0.0 0.0 0.0 0.0
echo "PASS: saved-map localization and round-trip navigation completed"
