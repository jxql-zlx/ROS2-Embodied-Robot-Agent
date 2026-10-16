#!/usr/bin/env bash
# Run unit tests for robot_control / robot_tools / robot_agent.
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/.."
cd ros2
source install/setup.bash
set -euo pipefail

for pkg in robot_control robot_tools robot_agent; do
  echo "== unit tests: $pkg =="
  (cd "src/$pkg" && python3 -m unittest discover -s test -v)
done
cd ..
echo "== unit tests: vision =="
python3 -m unittest discover -s vision/test -v
echo "ALL TESTS PASSED"
