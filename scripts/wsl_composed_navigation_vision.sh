#!/usr/bin/env bash
# End-to-end hierarchical Nav2 -> real YOLO cup detection -> approach check.
# Scenarios: success, navigation_failure, target_absent, blocked,
# vision_dropout, range_dropout.
source /opt/ros/humble/setup.bash
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPTS_DIR/.." && pwd)"
ROS2_DIR="$REPO_ROOT/ros2"
SCENARIO="${1:-success}"
case "$SCENARIO" in
  success|navigation_failure|target_absent|blocked|vision_dropout|range_dropout) ;;
  *) echo "usage: $0 [success|navigation_failure|target_absent|blocked|vision_dropout|range_dropout]"; exit 2 ;;
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
if scenario == "blocked":
    model = ET.SubElement(world, "model", name="composed_approach_obstacle")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "0.3 0 0.13 0 0 0"
    link = ET.SubElement(model, "link", name="bar")
    for kind in ("collision", "visual"):
        geometry = ET.SubElement(ET.SubElement(link, kind, name=kind), "geometry")
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = "0.05 0.2 0.01"
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
FAULT_PID=""

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
  stop_group "$FAULT_PID"
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
SCAN_TOPIC=/scan
APPROACH_STOP_DISTANCE=0.65
if [ "$SCENARIO" = vision_dropout ] || [ "$SCENARIO" = range_dropout ]; then
  APPROACH_STOP_DISTANCE=0.4
fi
if [ "$SCENARIO" = range_dropout ]; then SCAN_TOPIC=/approach_test/scan; fi
setsid ros2 run robot_agent robot_agent_node --ros-args -p mode:=hierarchical \
  -p "tool_scan_topic:=${SCAN_TOPIC}" -p "approach_stop_distance:=${APPROACH_STOP_DISTANCE}" \
  > "$LOG_DIR/agent.log" 2>&1 &
AGENT_PID=$!
setsid python3 "$REPO_ROOT/vision/ros2_vision_node.py" \
  --ros-args -p model_path:="$REPO_ROOT/models/yolov8n.pt" \
  -p camera_topic:=/camera/image_raw -p conf:=0.25 \
  > "$LOG_DIR/vision.log" 2>&1 &
VISION_PID=$!
if [ "$SCENARIO" = vision_dropout ] || [ "$SCENARIO" = range_dropout ]; then
  setsid python3 "$SCRIPTS_DIR/approach_fault_injector.py" "$SCENARIO" "$VISION_PID" \
    > "$LOG_DIR/fault_injector.log" 2>&1 &
  FAULT_PID=$!
fi

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
  blocked)
    TASK="导航到 0,0,0，然后找杯子并靠近"
    expected_rc=2
    ;;
  vision_dropout|range_dropout)
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

BASELINE=$(python3 "$SCRIPTS_DIR/odom_pose.py")
python3 - "$SCENARIO" "$LOG_DIR/task.log" "$LOG_DIR/stages" <<'PY'
import json
import math
import pathlib
import re
import sys
import time

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
    if (detected.get("stale") is not False or
            not isinstance(detected.get("observed_at"), (int, float)) or
            not math.isfinite(detected["observed_at"]) or
            detected["observed_at"] <= 0 or detected["observed_at"] > time.time() or
            not isinstance(detected.get("age_sec"), (int, float)) or
            not math.isfinite(detected["age_sec"]) or not 0 <= detected["age_sec"] <= 2.5):
        raise SystemExit(f"FAIL: navigation handed off missing, malformed, or stale metadata: {detected}")
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
elif scenario in ("blocked", "vision_dropout", "range_dropout"):
    if (len(results) != 3 or results[0].get("task_id") != 1 or
            results[1].get("task_id") != 2 or results[2].get("task_id") != 3 or
            not results[0].get("success") or not results[1].get("success") or
            results[2].get("success")):
        raise SystemExit(f"FAIL: expected nav and vision success followed by approach failure: {results}")
    response = next((line.partition(":")[2].strip() for line in text.splitlines()
                     if line.startswith("response")), "")
    expected = {
        "blocked": "前方距离不足",
        "vision_dropout": "运动后未收到新视觉帧",
        "range_dropout": "运动后未获得稳定的前方测距",
    }[scenario]
    if expected not in response:
        raise SystemExit(f"FAIL: wrong {scenario} response: {response}")
    if scenario == "blocked":
        approach = results[2].get("data", {}).get("approach", {})
        if (not approach.get("available") or not isinstance(approach.get("distance"), (int, float))
                or approach["distance"] >= 0.4):
            raise SystemExit(f"FAIL: blocked scenario did not verify close obstacle: {approach}")
print(f"PASS: {scenario}; stage logs: {out}")
PY

POST_TASK=$(python3 "$SCRIPTS_DIR/odom_pose.py")
sleep 1
FINAL=$(python3 "$SCRIPTS_DIR/odom_pose.py")
python3 - "$SCENARIO" "$BASELINE" "$POST_TASK" "$FINAL" <<'PY'
import math
import re
import sys
def pose(text):
    match = re.search(r"x=([-.0-9]+) y=([-.0-9]+) yaw=([-.0-9]+)", text)
    return tuple(map(float, match.groups())) if match else None
scenario = sys.argv[1]
baseline, before, after = map(pose, sys.argv[2:])
if baseline is None or before is None or after is None:
    raise SystemExit("FAIL: could not read post-task odometry")
drift = math.hypot(after[0] - before[0], after[1] - before[1])
yaw = math.atan2(math.sin(after[2] - before[2]), math.cos(after[2] - before[2]))
if drift > 0.02 or abs(yaw) > 0.03:
    raise SystemExit(f"FAIL: robot did not remain stopped: drift={drift:.3f}m yaw={yaw:.3f}rad")
travel = math.hypot(after[0] - baseline[0], after[1] - baseline[1])
if scenario == "blocked" and travel > 0.02:
    raise SystemExit(f"FAIL: robot moved toward blocked target: {travel:.3f}m")
print(f"PASS: post-task stop verified; drift={drift:.3f}m yaw={yaw:.3f}rad")
PY
echo "PASS: composed hierarchical navigation / YOLO / approach scenario=$SCENARIO"
echo "Logs retained in: $LOG_DIR"
