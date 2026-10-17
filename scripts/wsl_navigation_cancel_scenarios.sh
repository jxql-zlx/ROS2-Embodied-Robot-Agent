#!/usr/bin/env bash
# Exercise Agent-driven Nav2 cancellation and immediate follow-up navigation.
source /opt/ros/humble/setup.bash
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/ros2/install/setup.bash"
set -euo pipefail

LOG_ROOT="${TMPDIR:-/tmp}/ros2_navigation_cancel"
mkdir -p "$LOG_ROOT"
LOG_DIR="$(mktemp -d "$LOG_ROOT/run.XXXXXX")"
echo "RUN_LOG_DIR: $LOG_DIR"
MAP_PREFIX="${TMPDIR:-/tmp}/ros2_navigation_smoke/smoke_map"
MODES=("$@")
if [ "${#MODES[@]}" -eq 0 ]; then
  MODES=(simple hierarchical)
fi
for mode in "${MODES[@]}"; do
  if [ "$mode" != simple ] && [ "$mode" != hierarchical ]; then
    echo "unknown mode: $mode (expected simple and/or hierarchical)"
    exit 2
  fi
done
if [ ! -f "${MAP_PREFIX}.yaml" ]; then
  echo "Saved smoke map missing; generating it with the navigation smoke workflow"
  bash "$REPO_ROOT/scripts/wsl_navigation_smoke.sh"
fi

SIM_PID=""
NAV_PID=""
CONTROL_PID=""
AGENT_PID=""
SCENARIO_PID=""
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
  stop_group "$SCENARIO_PID"
  stop_group "$AGENT_PID"
  stop_group "$CONTROL_PID"
  stop_group "$NAV_PID"
  stop_group "$SIM_PID"
}
trap cleanup EXIT INT TERM

wait_topic() {
  python3 "$REPO_ROOT/scripts/wait_topic.py" "$1" "$2" --timeout "$3" "${@:4}"
}
wait_service() {
  local service="$1"
  for _ in $(seq 1 60); do
    if ros2 service list | grep -Fxq "$service"; then
      return 0
    fi
    sleep 0.5
  done
  echo "TIMEOUT: service $service is unavailable"
  return 1
}
setsid ros2 launch robot_bringup sim.launch.py gui:=false rviz:=false \
  > "$LOG_DIR/sim.log" 2>&1 &
SIM_PID=$!
wait_topic /clock rosgraph_msgs/msg/Clock 60 --sensor-qos
wait_topic /odom nav_msgs/msg/Odometry 60
wait_topic /scan sensor_msgs/msg/LaserScan 60 --sensor-qos

setsid ros2 launch robot_bringup navigation.launch.py mode:=localization \
  "map:=${MAP_PREFIX}.yaml" > "$LOG_DIR/navigation.log" 2>&1 &
NAV_PID=$!
python3 "$REPO_ROOT/scripts/navigation_smoke_client.py" check-localization
setsid ros2 run robot_control robot_control_node > "$LOG_DIR/control.log" 2>&1 &
CONTROL_PID=$!

for mode in "${MODES[@]}"; do
  echo "[Nav2 cancellation scenario] mode=$mode"
  setsid ros2 run robot_agent robot_agent_node --ros-args -p "mode:=${mode}" \
    > "$LOG_DIR/agent_${mode}.log" 2>&1 &
  AGENT_PID=$!
  wait_service /task_execute
  setsid python3 "$REPO_ROOT/scripts/navigation_cancel_scenario.py" "$mode" \
    > "$LOG_DIR/scenario_${mode}.log" 2>&1 &
  SCENARIO_PID=$!
  if ! wait "$SCENARIO_PID"; then
    SCENARIO_PID=""
    echo "FAIL: $mode cancellation scenario failed"
    echo "Logs retained in: $LOG_DIR"
    tail -100 "$LOG_DIR/scenario_${mode}.log"
    tail -80 "$LOG_DIR/agent_${mode}.log"
    tail -80 "$LOG_DIR/navigation.log"
    exit 1
  fi
  SCENARIO_PID=""
  cat "$LOG_DIR/scenario_${mode}.log"
  stop_group "$AGENT_PID"
  AGENT_PID=""
done

echo "PASS: Agent Nav2 cancellation scenarios completed (modes: ${MODES[*]})"
echo "Logs retained in: $LOG_DIR"
