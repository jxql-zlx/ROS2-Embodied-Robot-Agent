#!/usr/bin/env bash
# Headless Vision -> Language -> Action demo:
#   Gazebo cup world + vision node (YOLO) + robot_agent (mock LLM).
#   Sends "找杯子" and verifies the robot moves toward the detected cup.
source /opt/ros/humble/setup.bash

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPTS_DIR/.." && pwd)"
ROS2_DIR="$REPO_ROOT/ros2"
AGENT_MODE="${1:-simple}"
TASK="${2:-找杯子}"
SCENARIO="${3:-center}"
case "$AGENT_MODE" in
  simple|hierarchical) ;;
  *) echo "usage: $0 [simple|hierarchical] [task] [scenario]"; exit 1 ;;
esac
case "$SCENARIO" in
  center|left|right|near|far|blocked|absent|vision_dropout|range_dropout) ;;
  *) echo "unknown scenario: $SCENARIO"; exit 1 ;;
esac

if [ "${APPROACH_SKIP_BUILD:-0}" != 1 ]; then
  bash "$SCRIPTS_DIR/wsl_build.sh" --packages-select robot_description robot_tools robot_agent robot_bringup || exit 1
fi
cd "$ROS2_DIR"
source install/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
set -uo pipefail

MODELS="$ROS2_DIR/src/robot_bringup/models"
WORLD="$ROS2_DIR/install/robot_bringup/share/robot_bringup/worlds/cup.world"
TEST_WORLD=$(mktemp /tmp/approach_world_XXXXXX.world)
python3 - "$WORLD" "$TEST_WORLD" "$SCENARIO" <<'PY'
import sys
import xml.etree.ElementTree as ET
tree = ET.parse(sys.argv[1])
world = tree.getroot().find("world")
cup = next(item for item in world.findall("include") if item.findtext("uri") == "model://cup")
if sys.argv[3] == "absent":
    world.remove(cup)
elif sys.argv[3] in ("left", "right"):
    y = 0.25 if sys.argv[3] == "left" else -0.25
    cup.find("pose").text = f"1.0 {y} 0 0 0 0"
elif sys.argv[3] in ("near", "far"):
    x = 0.8 if sys.argv[3] == "near" else 1.3
    cup.find("pose").text = f"{x} 0 0 0 0 0"
elif sys.argv[3] == "blocked":
    # A low crossbar at laser height: an obstacle while the cup remains visible.
    model = ET.SubElement(world, "model", name="approach_obstacle")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "0.3 0 0.13 0 0 0"
    link = ET.SubElement(model, "link", name="bar")
    for kind in ("collision", "visual"):
        geometry = ET.SubElement(ET.SubElement(link, kind, name=kind), "geometry")
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = "0.05 0.2 0.01"
tree.write(sys.argv[2])
PY
WORLD="$TEST_WORLD"
SCAN_TOPIC=/scan
if [ "$SCENARIO" = range_dropout ]; then SCAN_TOPIC=/approach_test/scan; fi
MODEL="$REPO_ROOT/models/yolov8n.pt"
LOG="/tmp/vision_action_demo_${AGENT_MODE}_${SCENARIO}.log"

export GAZEBO_MODEL_PATH="$MODELS:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""

setsid ros2 launch robot_bringup demo.launch.py world:="$WORLD" gui:=false rviz:=false agent_mode:="$AGENT_MODE" tool_scan_topic:="$SCAN_TOPIC" > "$LOG" 2>&1 &
LAUNCH_PID=$!

setsid python3 "$REPO_ROOT/vision/ros2_vision_node.py" \
  --ros-args -p model_path:="$MODEL" -p camera_topic:=/camera/image_raw -p conf:=0.25 \
  > /tmp/vision_action_node.log 2>&1 &
VISION_PID=$!
FAULT_PID=""
if [ "$SCENARIO" = vision_dropout ] || [ "$SCENARIO" = range_dropout ]; then
  setsid python3 "$SCRIPTS_DIR/approach_fault_injector.py" "$SCENARIO" "$VISION_PID" > /tmp/approach_fault.log 2>&1 &
  FAULT_PID=$!
fi

cleanup() {
  if [ -n "$FAULT_PID" ]; then kill -TERM -- -"$FAULT_PID" 2>/dev/null || true; fi
  kill -INT -- -"$VISION_PID" 2>/dev/null || true
  kill -INT -- -"$LAUNCH_PID" 2>/dev/null || true
  sleep 2
  kill -TERM -- -"$VISION_PID" 2>/dev/null || true
  kill -TERM -- -"$LAUNCH_PID" 2>/dev/null || true
  wait "$VISION_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  if [ -n "$FAULT_PID" ]; then wait "$FAULT_PID" 2>/dev/null || true; fi
  rm -f -- "$TEST_WORLD"
}
trap cleanup EXIT

echo ">> waiting for topics ..."
python3 "$SCRIPTS_DIR/wait_topic.py" /camera/image_raw sensor_msgs/msg/Image --timeout 90 \
  || { tail -60 "$LOG"; exit 1; }
python3 "$SCRIPTS_DIR/wait_topic.py" /vision/detections std_msgs/msg/String --timeout 60 \
  || { tail -60 /tmp/vision_action_node.log; exit 1; }
python3 "$SCRIPTS_DIR/wait_topic.py" /odom nav_msgs/msg/Odometry --timeout 60 || exit 1

echo ">> baseline pose"
BASELINE=$(python3 "$SCRIPTS_DIR/odom_pose.py")
echo "$BASELINE"

echo ">> mode=$AGENT_MODE scenario=$SCENARIO task=$TASK"
TASK_OUTPUT=$(ros2 run robot_agent send_task "$TASK" --service --timeout 40 2>&1)
TASK_RESULT=$?
echo "$TASK_OUTPUT"
EXPECTED_RESULT=0
case "$SCENARIO" in absent|blocked|vision_dropout|range_dropout) EXPECTED_RESULT=2 ;; esac
if [ "$TASK_RESULT" -ne "$EXPECTED_RESULT" ]; then
  echo "FAIL: approach task did not complete successfully"
  tail -30 "$LOG"
  exit 1
fi

POST_TASK=$(python3 "$SCRIPTS_DIR/odom_pose.py")
echo ">> checking robot remains stopped ..."
sleep 2

echo ">> final pose"
FINAL=$(python3 "$SCRIPTS_DIR/odom_pose.py")
echo "$FINAL"

python3 - "$BASELINE" "$FINAL" "$TASK_OUTPUT" "$SCENARIO" "$POST_TASK" <<'PY'
import json
import math
import re
import sys


def parse(line):
    m = re.search(r"x=([-\d.]+) y=([-\d.]+)", line)
    return (float(m.group(1)), float(m.group(2))) if m else None


b = parse(sys.argv[1])
f = parse(sys.argv[2])
if b is None or f is None:
    print("FAIL: could not parse odom poses")
    sys.exit(1)

dist = math.hypot(f[0] - b[0], f[1] - b[1])
print(f"displacement={dist:.3f} m (threshold=0.05)")
post = parse(sys.argv[5])
if post is None or math.hypot(f[0] - post[0], f[1] - post[1]) > 0.02:
    raise SystemExit("FAIL: robot kept moving after task completion")
final_yaw = float(re.search(r"yaw=([-\d.]+)", sys.argv[2]).group(1))
post_yaw = float(re.search(r"yaw=([-\d.]+)", sys.argv[5]).group(1))
yaw_change = math.atan2(math.sin(final_yaw - post_yaw), math.cos(final_yaw - post_yaw))
if abs(yaw_change) > 0.03:
    raise SystemExit("FAIL: robot kept rotating after task completion")
scenario = sys.argv[4]
if scenario in ("absent", "blocked", "vision_dropout", "range_dropout"):
    limit = 0.02 if scenario in ("absent", "blocked") else 0.25
    reasons = {"absent": ("没有发现", "no target found"),
               "vision_dropout": ("未收到新视觉帧",),
               "blocked": ("前方距离不足",),
               "range_dropout": ("未获得稳定",)}
    if dist > limit or not any(reason in sys.argv[3] for reason in reasons[scenario]):
        raise SystemExit("FAIL: fault response or travel bound invalid")
    print(f"PASS: {scenario} failed explicitly and robot stopped")
    raise SystemExit(0)
action_line = next((line for line in sys.argv[3].splitlines() if line.startswith("action_json:")), "")
data = json.loads(action_line.split(":", 1)[1]) if action_line else {}
if isinstance(data, list):
    approaches = [entry["data"]["approach"] for entry in data
                  if entry.get("success") and "approach" in entry.get("data", {})]
    data = approaches[-1] if approaches else {}
distance = data.get("distance")
if dist >= 0.05 and data.get("available") and isinstance(distance, (int, float)) and 0.4 <= distance <= 0.65:
    yaw = final_yaw
    if scenario in ("left", "right") and yaw * (1 if scenario == "left" else -1) < 0.1:
        raise SystemExit("FAIL: robot did not turn toward the side target")
    print(f"PASS: approach completed at measured standoff {distance:.3f} m")
    sys.exit(0)
print(f"FAIL: movement or final range invalid: {distance}")
sys.exit(2)
PY
