#!/usr/bin/env bash
# Save the live /map published during a mapping session as an image + YAML.
source /opt/ros/humble/setup.bash
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 /absolute/path/without-extension" >&2
  exit 2
fi

map_prefix="$1"
if [[ "$map_prefix" != /* ]]; then
  echo "Map output prefix must be an absolute path (for example /mnt/f/maps/room)" >&2
  exit 2
fi

mkdir -p "$(dirname "$map_prefix")"
ros2 run nav2_map_server map_saver_cli -t /map -f "$map_prefix" \
  --ros-args -p use_sim_time:=true

for output in "${map_prefix}.yaml" "${map_prefix}.pgm"; do
  if [ ! -s "$output" ]; then
    echo "Map saver did not produce a non-empty file: $output" >&2
    exit 1
  fi
done

echo "Saved map metadata: ${map_prefix}.yaml"
echo "Saved map image:    ${map_prefix}.pgm"
