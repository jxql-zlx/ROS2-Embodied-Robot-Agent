#!/usr/bin/env bash
# Headless Vision -> Language -> Action demo:
#   Gazebo cup world + vision node (YOLO) + robot_agent (mock LLM).
#   Sends "找杯子" and verifies the robot moves toward the detected cup.
source /opt/ros/humble/setup.bash

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPTS_DIR/.." && pwd)"
ROS2_DIR="$REPO_ROOT/ros2"

bash "$SCRIPTS_DIR/wsl_build.sh" --packages-select robot_tools robot_agent || exit 1
cd "$ROS2_DIR"
source install/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
set -uo pipefail

MODELS="$ROS2_DIR/src/robot_bringup/models"
WORLD="$ROS2_DIR/install/robot_bringup/share/robot_bringup/worlds/cup.world"
MODEL="$REPO_ROOT/models/yolov8n.pt"
LOG=/tmp/vision_action_demo.log

export GAZEBO_MODEL_PATH="$MODELS:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""

setsid ros2 launch robot_bringup demo.launch.py world:="$WORLD" gui:=false rviz:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!

setsid python3 "$REPO_ROOT/vision/ros2_vision_node.py" \
  --ros-args -p model_path:="$MODEL" -p camera_topic:=/camera/image_raw -p conf:=0.25 \
  > /tmp/vision_action_node.log 2>&1 &
VISION_PID=$!

cleanup() {
  kill -INT -- -"$VISION_PID" 2>/dev/null || true
  kill -INT -- -"$LAUNCH_PID" 2>/dev/null || true
  sleep 2
  kill -TERM -- -"$VISION_PID" 2>/dev/null || true
  kill -TERM -- -"$LAUNCH_PID" 2>/dev/null || true
  wait "$VISION_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
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

echo ">> sending task: 找杯子"
TASK_OUTPUT=$(ros2 run robot_agent send_task "找杯子" --service --timeout 40 2>&1)
TASK_RESULT=$?
echo "$TASK_OUTPUT"
if [ "$TASK_RESULT" -ne 0 ]; then
  echo "FAIL: approach task did not complete successfully"
  tail -30 "$LOG"
  exit 1
fi

echo ">> waiting for motion ..."
sleep 6

echo ">> final pose"
FINAL=$(python3 "$SCRIPTS_DIR/odom_pose.py")
echo "$FINAL"

python3 - "$BASELINE" "$FINAL" "$TASK_OUTPUT" <<'PY'
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
action_line = next((line for line in sys.argv[3].splitlines() if line.startswith("action_json:")), "")
data = json.loads(action_line.split(":", 1)[1]) if action_line else {}
distance = data.get("distance")
if dist >= 0.05 and data.get("available") and isinstance(distance, (int, float)) and 0.4 <= distance <= 0.65:
    print(f"PASS: approach completed at measured standoff {distance:.3f} m")
    sys.exit(0)
print(f"FAIL: movement or final range invalid: {distance}")
sys.exit(2)
PY
