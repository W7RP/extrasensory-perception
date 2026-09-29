#!/usr/bin/env bash
# Fly the scenario once: the entity starts walking and every agent flies its
# route, takes off, and lands. Needs scripts/sim.sh (and, for anything to be
# tracked, scripts/stack.sh) running. Exit code 0 if every agent landed.
#
#   scripts/fly.sh                         # terminal 3
set -euo pipefail
source "$(dirname "$0")/env.sh"
source "$REPO_ROOT/scripts/lib/demo_common.sh"

scenario="${1:-$REPO_ROOT/sim/scenarios/two_agent_wall.yaml}"
COOP_LOG_DIR="${COOP_LOG_DIR:-/tmp/coop-fly}"
export COOP_LOG_DIR
mkdir -p "$COOP_LOG_DIR"
trap 'for p in "${demo_pids[@]}"; do kill "$p" 2>/dev/null || true; done' EXIT
trap 'exit 130' INT TERM

eval "$(python3 "$REPO_ROOT/scripts/lib/scenario.py" env "$scenario")"
params="$COOP_LOG_DIR/params"
python3 "$REPO_ROOT/scripts/lib/scenario.py" params "$scenario" "$params" >/dev/null

python3 "$REPO_ROOT/scripts/entity_mover.py" --scenario "$scenario" >"$COOP_LOG_DIR/entity_mover.log" 2>&1 &
demo_pids+=($!)
offboard=()
for n in "${SC_AGENT_IDS[@]}"; do
  demo_start_node "$COOP_LOG_DIR/offboard_$n.log" agent_offboard offboard_agent --ros-args \
    -r __ns:="/agent_$n" \
    --params-file "$(ros2 pkg prefix agent_offboard)/share/agent_offboard/config/offboard.yaml" \
    --params-file "$params/offboard_$n.yaml"
  offboard+=("${demo_pids[-1]}")
done
log "agents flying (logs: $COOP_LOG_DIR/offboard_*.log)"
rc=0
for i in "${!offboard[@]}"; do
  if wait "${offboard[i]}"; then ok "agent ${SC_AGENT_IDS[i]} landed"; else warn "agent ${SC_AGENT_IDS[i]} did not complete"; rc=1; fi
done
exit "$rc"
