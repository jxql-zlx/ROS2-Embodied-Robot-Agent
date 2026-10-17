#!/usr/bin/env bash
# End-to-end hierarchical Nav2 -> real YOLO cup detection -> approach check.
# Scenarios: success, navigation_failure, target_absent.
source /opt/ros/humble/setup.bash
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPTS_DIR/.." && pwd)"
ROS2_DIR="$REPO_ROOT/ros2"
SCENARIO="${1:-success}"
case "$SCENARIO" in
  success|navigation_failure|target_absent) ;;
  *) echo "usage: $0 [success|navigation_failure|target_absent]"; exit 2 ;;
esac

source "$ROS2_DIR/install/setup.bash"
source /usr/share/gazebo/setup.sh 2>/dev/null || true
set -euo pipefail
MAP_PREFIX="${TMPDIR:-/tmp}/ros2_navigation_smoke/smoke_map"
if [ ! -f "${MAP_PREFIX}.yaml" ]; then
  echo "Saved smoke map missing; generating it with the navigation smoke workflow"
  bash "$SCRIPTS_DIR/wsl_navigation_smoke.sh"
fi

LOG_ROOT="${TMPDIR:-/tmp}/ros2_composed_navigation"
mkdir -p "$LOG_ROOT"
LOG_DIR="$(mktemp -d "$LOG_ROOT/${SCENARIO}.XXXXXX")"
echo "RUN_LOG_DIR: $LOG_DIR"

WORLD_SOURCE="$ROS2_DIR/install/robot_bringup/share/robot_bringup/worlds/simple_world.world"
WORLD="$LOG_DIR/composed.world"
python3 - "$WORLD_SOURCE" "$WORLD" "$SCENARIO" <<'PY'
import sys
import xml.etree.ElementTree as ET

source, destination, scenario = sys.argv[1:]
tree = ET.parse(source)
world = tree.getroot().find("world")
if scenario != "target_absent":
    include = ET.SubElement(world, "include")
    ET.SubElement(include, "uri").text = "model://cup"
    ET.SubElement(include, "pose").text = "1.35 0 0 0 0 0"
tree.write(destination)
PY

MODELS="$ROS2_DIR/src/robot_bringup/models"
export GAZEBO_MODEL_PATH="$MODELS:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
SIM_PID=""
NAV_PID=""
CONTROL_PID=""
AGENT_PID=""
VISION_PID=""

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
  stop_group "$VISION_PID"
  stop_group "$AGENT_PID"
  stop_group "$CONTROL_PID"
  stop_group "$NAV_PID"
  stop_group "$SIM_PID"
}
trap cleanup EXIT INT TERM

wait_service() {
  local service="$1"
  for _ in $(seq 1 60); do
    if ros2 service list | grep -Fxq "$service"; then return 0; fi
    sleep 0.5
  done
  echo "TIMEOUT: service $service is unavailable"
  return 1
}

setsid ros2 launch robot_bringup sim.launch.py world:="$WORLD" gui:=false rviz:=false \
  > "$LOG_DIR/sim.log" 2>&1 &
SIM_PID=$!
python3 "$SCRIPTS_DIR/wait_topic.py" /clock rosgraph_msgs/msg/Clock --timeout 60 --sensor-qos
python3 "$SCRIPTS_DIR/wait_topic.py" /odom nav_msgs/msg/Odometry --timeout 60
python3 "$SCRIPTS_DIR/wait_topic.py" /scan sensor_msgs/msg/LaserScan --timeout 60 --sensor-qos

setsid ros2 launch robot_bringup navigation.launch.py mode:=localization \
  "map:=${MAP_PREFIX}.yaml" > "$LOG_DIR/navigation.log" 2>&1 &
NAV_PID=$!
python3 "$SCRIPTS_DIR/navigation_smoke_client.py" check-localization \
  > "$LOG_DIR/localization_check.log" 2>&1
setsid ros2 run robot_control robot_control_node > "$LOG_DIR/control.log" 2>&1 &
CONTROL_PID=$!
setsid ros2 run robot_agent robot_agent_node --ros-args -p mode:=hierarchical \
  > "$LOG_DIR/agent.log" 2>&1 &
AGENT_PID=$!
setsid python3 "$REPO_ROOT/vision/ros2_vision_node.py" \
  --ros-args -p model_path:="$REPO_ROOT/models/yolov8n.pt" \
  -p camera_topic:=/camera/image_raw -p conf:=0.25 \
  > "$LOG_DIR/vision.log" 2>&1 &
VISION_PID=$!

python3 "$SCRIPTS_DIR/wait_topic.py" /camera/image_raw sensor_msgs/msg/Image --timeout 90
python3 "$SCRIPTS_DIR/wait_topic.py" /vision/detections std_msgs/msg/String --timeout 60
wait_service /task_execute

case "$SCENARIO" in
  navigation_failure)
    TASK="导航到 999,999,0，然后找杯子并靠近"
    expected_rc=2
    ;;
  target_absent)
    TASK="导航到 0.2,0,0，然后找杯子并靠近"
    expected_rc=2
    ;;
  *)
    TASK="导航到 0.2,0,0，然后找杯子并靠近"
    expected_rc=0
    ;;
esac
echo "SCENARIO=$SCENARIO TASK=$TASK"
if ros2 run robot_agent send_task "$TASK" --service --timeout 180 \
  > "$LOG_DIR/task.log" 2>&1; then
  task_rc=0
else
  task_rc=$?
fi
cat "$LOG_DIR/task.log"
if [ "$task_rc" -ne "$expected_rc" ]; then
  echo "FAIL: task exit code $task_rc; expected $expected_rc"
  echo "Logs retained in: $LOG_DIR"
  exit 1
fi

python3 - "$SCENARIO" "$LOG_DIR/task.log" "$LOG_DIR/stages" <<'PY'
import json
import pathlib
import sys

scenario, task_log, output_dir = sys.argv[1:]
text = pathlib.Path(task_log).read_text(encoding="utf-8")
line = next((line.partition(":")[2].strip() for line in text.splitlines()
             if line.startswith("action_json:")), None)
if not line:
    raise SystemExit("FAIL: task response has no action_json")
results = json.loads(line)
out = pathlib.Path(output_dir)
out.mkdir(parents=True, exist_ok=True)
names = {1: "navigation", 2: "vision", 3: "approach", -1: "task_summary"}
for result in results:
    stage = names.get(result.get("task_id"), f"task_{result.get('task_id')}")
    (out / f"{result.get('task_id'):02d}_{stage}.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

if scenario == "success":
    if len(results) != 3 or not all(item.get("success") for item in results):
        raise SystemExit(f"FAIL: expected three successful stages, got {results}")
    detected = results[1].get("data", {}).get("vision", {})
    if (not detected.get("found") or not detected.get("objects") or
            not any(obj.get("name", "").lower() == "cup" for obj in detected["objects"])):
        raise SystemExit(f"FAIL: YOLO did not hand off a cup detection: {detected}")
    if detected.get("stale"):
        raise SystemExit(f"FAIL: navigation handed off a stale detection: {detected}")
    approach = results[2].get("data", {}).get("approach", {})
    distance = approach.get("distance")
    if not approach.get("available") or not isinstance(distance, (int, float)) or not 0.4 <= distance <= 0.65:
        raise SystemExit(f"FAIL: approach did not verify the stop distance: {approach}")
elif scenario == "navigation_failure":
    if (len(results) != 2 or results[0].get("task_id") != 1 or
            results[0].get("success") or results[-1].get("task_id") != -1):
        raise SystemExit(f"FAIL: navigation failure did not stop all dependent stages: {results}")
elif scenario == "target_absent":
    if (len(results) != 3 or results[0].get("task_id") != 1 or
            results[1].get("task_id") != 2 or results[-1].get("task_id") != -1 or
            not results[0].get("success") or results[1].get("success")):
        raise SystemExit(f"FAIL: absent target did not stop approach: {results}")
    if results[1].get("data", {}).get("found"):
        raise SystemExit(f"FAIL: absent target was reported as found: {results[1]}")
print(f"PASS: {scenario}; stage logs: {out}")
PY

if [ "$SCENARIO" = success ]; then
  POST_TASK=$(python3 "$SCRIPTS_DIR/odom_pose.py")
  sleep 1
  FINAL=$(python3 "$SCRIPTS_DIR/odom_pose.py")
  python3 - "$POST_TASK" "$FINAL" <<'PY'
import math
import re
import sys
def pose(text):
    match = re.search(r"x=([-.0-9]+) y=([-.0-9]+) yaw=([-.0-9]+)", text)
    return tuple(map(float, match.groups())) if match else None
before, after = map(pose, sys.argv[1:])
if before is None or after is None:
    raise SystemExit("FAIL: could not read post-task odometry")
drift = math.hypot(after[0] - before[0], after[1] - before[1])
yaw = math.atan2(math.sin(after[2] - before[2]), math.cos(after[2] - before[2]))
if drift > 0.02 or abs(yaw) > 0.03:
    raise SystemExit(f"FAIL: robot did not remain stopped: drift={drift:.3f}m yaw={yaw:.3f}rad")
print(f"PASS: post-task stop verified; drift={drift:.3f}m yaw={yaw:.3f}rad")
PY
fi
echo "PASS: composed hierarchical navigation / YOLO / approach scenario=$SCENARIO"
echo "Logs retained in: $LOG_DIR"
