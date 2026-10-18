#!/usr/bin/env bash
# Run Gazebo scenario regressions for both agent modes, building once.
source /opt/ros/humble/setup.bash
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$SCRIPTS_DIR/wsl_build.sh" --packages-select robot_interfaces robot_description robot_tools robot_agent robot_bringup || exit 1
set -euo pipefail
if [ "$#" -eq 0 ]; then set -- simple hierarchical; fi
scenario_names="${APPROACH_SCENARIOS:-left right near far blocked absent vision_dropout range_dropout}"
scenario_names="${scenario_names//,/ }"
read -r -a scenarios <<< "$scenario_names"
LOG_ROOT="${TMPDIR:-/tmp}/approach_scenarios"
mkdir -p "$LOG_ROOT"
LOG_DIR="$(mktemp -d "$LOG_ROOT/run.XXXXXX")"
echo "RUN_LOG_DIR: $LOG_DIR"
for mode in "$@"; do
  for scenario in "${scenarios[@]}"; do
    echo ">> validating $mode / $scenario"
    log="$LOG_DIR/${mode}_${scenario}.log"
    if APPROACH_SKIP_BUILD=1 bash "$SCRIPTS_DIR/wsl_vision_action_demo.sh" "$mode" "找杯子" "$scenario" > "$log" 2>&1; then
      tail -8 "$log"
    else
      tail -45 "$log"
      exit 1
    fi
  done
done
echo "ALL APPROACH SCENARIOS PASSED; logs: $LOG_DIR"
