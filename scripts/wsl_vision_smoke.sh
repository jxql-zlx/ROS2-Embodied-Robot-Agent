#!/usr/bin/env bash
# Headless vision integration test:
#   Gazebo camera (/camera/image_raw) -> ros2_vision_node (YOLO) -> /vision/detections
# Verifies the full perception pipeline publishes a valid Scene JSON.
#
# Usage (from repo root or anywhere):
#   scripts/wsl_vision_smoke.sh
source /opt/ros/humble/setup.bash

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPTS_DIR/.." && pwd)"
ROS2_DIR="$REPO_ROOT/ros2"
MODEL="$REPO_ROOT/models/yolov8n.pt"
LOG=/tmp/vision_smoke.log

cd "$ROS2_DIR"
source install/setup.bash
set -uo pipefail

bash "$SCRIPTS_DIR/wsl_cleanup.sh" >/dev/null 2>&1 || true

echo ">> launching sim (headless) ..."
ros2 launch robot_bringup sim.launch.py gui:=false rviz:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT "${VISION_PID:-}" 2>/dev/null || true
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 2
  bash "$SCRIPTS_DIR/wsl_cleanup.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo ">> waiting for /camera/image_raw ..."
python3 "$SCRIPTS_DIR/wait_topic.py" /camera/image_raw sensor_msgs/msg/Image --timeout 90 \
  || { echo "FAIL: camera never appeared"; tail -80 "$LOG"; exit 1; }

echo ">> launching vision node (YOLO, local weights) ..."
python3 "$REPO_ROOT/vision/ros2_vision_node.py" \
  --ros-args -p use_sim_time:=true -p model_path:="$MODEL" -p camera_topic:=/camera/image_raw -p conf:=0.25 \
  > /tmp/vision_node.log 2>&1 &
VISION_PID=$!

echo ">> waiting for /vision/detections ..."
python3 "$SCRIPTS_DIR/wait_topic.py" /vision/detections std_msgs/msg/String --timeout 60 \
  || { echo "FAIL: /vision/detections never appeared"; echo '--- vision node log ---'; tail -60 /tmp/vision_node.log; tail -40 "$LOG"; exit 1; }
python3 "$SCRIPTS_DIR/wait_fresh_vision.py" \
  || { echo 'FAIL: no verified fresh camera timestamp'; tail -60 /tmp/vision_node.log; exit 1; }

echo ">> grabbing one detection message ..."
python3 - <<'PY'
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node("detections_check")
got = []
node.create_subscription(String, "/vision/detections", lambda m: got.append(m), 10)
deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=20)
while node.get_clock().now() < deadline and not got:
    rclpy.spin_once(node, timeout_sec=0.2)

if not got:
    print("FAIL: no detection message")
    rclpy.shutdown()
    raise SystemExit(1)

raw = got[0].data
try:
    scene = json.loads(raw)
except Exception as exc:
    print("FAIL: invalid JSON:", exc)
    print("raw:", raw[:500])
    rclpy.shutdown()
    raise SystemExit(1)

print("OK scene keys:", sorted(scene.keys()))
print("image:", scene.get("image_width"), "x", scene.get("image_height"), "source:", scene.get("source"))
print("objects:", len(scene.get("objects", [])))
for o in scene.get("objects", []):
    print(" -", o.get("name"), o.get("confidence"), o.get("bbox"), o.get("center"))
rclpy.shutdown()
PY

echo "PASS: /vision/detections verified"
