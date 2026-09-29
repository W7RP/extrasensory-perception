#!/usr/bin/env bash
# Start the perception stack against a running scripts/sim.sh: per agent an
# ESKF and a synthetic detector, then the fused tracker and one single-agent
# baseline tracker per agent. Stays in the foreground; Ctrl+C stops it.
#
#   scripts/stack.sh                       # terminal 2, after scripts/sim.sh
#   scripts/stack.sh --rviz                # plus RViz (fused tracks, building, truth)
#   POSE_SOURCE=ground_truth scripts/stack.sh   # detectors use perfect navigation
set -euo pipefail
source "$(dirname "$0")/env.sh"
source "$REPO_ROOT/scripts/lib/demo_common.sh"

rviz=0
scenario="$REPO_ROOT/sim/scenarios/two_agent_wall.yaml"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rviz) rviz=1 ;;
    --scenario) scenario="$(realpath "$2")"; shift ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done
COOP_LOG_DIR="${COOP_LOG_DIR:-/tmp/coop-stack}"
export COOP_LOG_DIR
mkdir -p "$COOP_LOG_DIR"
trap demo_cleanup_nodes EXIT
trap 'exit 130' INT TERM
# Only this script's nodes: the simulator belongs to sim.sh.
demo_cleanup_nodes() {
  local pid
  for pid in "${demo_pids[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}

eval "$(python3 "$REPO_ROOT/scripts/lib/scenario.py" env "$scenario")"
params="$COOP_LOG_DIR/params"
python3 "$REPO_ROOT/scripts/lib/scenario.py" params "$scenario" "$params" >/dev/null
if [[ -n "${POSE_SOURCE:-}" ]]; then
  sed -i "s/^    pose_source: .*/    pose_source: $POSE_SOURCE/" "$params"/detector_*.yaml
fi
demo_wait_for_message /sim/ground_truth tf2_msgs/msg/TFMessage 10 \
  || die "no simulator running: start scripts/sim.sh first"
stack_start "$params" "$COOP_LOG_DIR"
stack_wait_ready
if ((rviz)); then
  "$(ros2 pkg prefix rviz2)/lib/rviz2/rviz2" -d "$REPO_ROOT/sim/rviz/phase1.rviz" \
    --ros-args -p use_sim_time:=true >"$COOP_LOG_DIR/rviz.log" 2>&1 &
  demo_pids+=($!)
fi
ok "stack running; node logs in $COOP_LOG_DIR. Ctrl+C to stop."
wait -n "${demo_pids[@]}" || true
warn "a node exited; stopping the stack"
