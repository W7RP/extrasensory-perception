#!/usr/bin/env bash
# Phase 1 demo, end to end: two agents fly their scripted routes past the
# building while the entity walks behind it; each agent detects it (synthetic
# detector, ESKF navigation), and the fused tracker and both single-agent
# baselines track it. Then everything is scored against ground truth.
#
#   scripts/demo_phase1.sh              # headless (default)
#   scripts/demo_phase1.sh --gui        # Gazebo GUI + RViz
#   POSE_SOURCE=ground_truth scripts/demo_phase1.sh   # detectors use perfect navigation
#
# Owns the whole lifecycle and assumes nothing is running: it first clears any
# stale Gazebo / PX4 / XRCE agent / project node, then starts the simulator,
# waits for each stage by polling for real messages (no fixed sleeps), flies,
# evaluates, and tears everything down on exit, including Ctrl-C.
#
# Output: logs/phase1_<timestamp>/ with every process's log, the parameter
# files used, the rosbag (coop_bag/), metrics.json, results.txt and
# phase1.png. Exit code 0 only if both agents completed their routes AND the
# fused track met the acceptance thresholds in scripts/eval_phase1.py.
set -euo pipefail
source "$(dirname "$0")/env.sh"
source "$REPO_ROOT/scripts/lib/demo_common.sh"

gui=0
scenario="$REPO_ROOT/sim/scenarios/two_agent_wall.yaml"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui) gui=1 ;;
    --headless) gui=0 ;;
    --scenario) scenario="$(realpath "$2")"; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done

demo_logdir phase1
logdir="$COOP_LOG_DIR"
trap demo_cleanup EXIT
# A trapped INT/TERM would run the handler and then carry on with the script
# (seen in quad-autonomy-sim). Exit instead; the EXIT trap cleans up once.
trap 'exit 130' INT TERM
log "logs -> $logdir"

# 1. Nothing from an earlier run may survive into this one.
demo_clear_stale

# 2. Parameters for every node, from the scenario.
eval "$(python3 "$REPO_ROOT/scripts/lib/scenario.py" env "$scenario")"
params="$logdir/params"
python3 "$REPO_ROOT/scripts/lib/scenario.py" params "$scenario" "$params" >/dev/null
if [[ -n "${POSE_SOURCE:-}" ]]; then
  sed -i "s/^    pose_source: .*/    pose_source: $POSE_SOURCE/" "$params"/detector_*.yaml
  warn "detectors use pose_source=$POSE_SOURCE"
fi

# 3. Simulator: world, agents, bridges.
sim_args=(--scenario "$scenario")
((gui)) && sim_args+=(--gui)
demo_start_sim "$logdir" "${sim_args[@]}"
log "waiting for the simulator"
demo_wait_for_message /clock rosgraph_msgs/msg/Clock 60
for n in "${SC_AGENT_IDS[@]}"; do
  demo_wait_for_message "/px4_$n/fmu/out/vehicle_status_v1" px4_msgs/msg/VehicleStatus 120
  ok "agent $n: PX4 <-> ROS 2 bridge up"
done
demo_wait_for_message /sim/ground_truth tf2_msgs/msg/TFMessage 60
ok "ground truth for the entity and every agent"

# 4. Perception stack, while the agents are still on the ground: each ESKF
#    aligns from ~1.2 s of still IMU data before it publishes.
stack_start "$params" "$logdir"
stack_wait_ready
if ((gui)); then
  "$(ros2 pkg prefix rviz2)/lib/rviz2/rviz2" -d "$REPO_ROOT/sim/rviz/phase1.rviz" \
    --ros-args -p use_sim_time:=true >"$logdir/rviz.log" 2>&1 &
  demo_pids+=($!)
fi

# 5. Record what the evaluation needs.
# shellcheck disable=SC2046
demo_start_bag "$logdir/coop_bag" $(stack_record_topics)
demo_wait_for_message /fusion/tracks coop_msgs/msg/TrackArray 10  # recorder subscribed meanwhile

# 6. The mission: the entity starts walking, both agents fly their routes.
python3 "$REPO_ROOT/scripts/entity_mover.py" --scenario "$scenario" >"$logdir/entity_mover.log" 2>&1 &
demo_pids+=($!)
offboard_pids=()
for n in "${SC_AGENT_IDS[@]}"; do
  demo_start_node "$logdir/offboard_$n.log" agent_offboard offboard_agent --ros-args \
    -r __ns:="/agent_$n" \
    --params-file "$(ros2 pkg prefix agent_offboard)/share/agent_offboard/config/offboard.yaml" \
    --params-file "$params/offboard_$n.yaml"
  offboard_pids+=("${demo_pids[-1]}")
done
log "agents flying (tail -f $logdir/offboard_*.log)"
flight_rc=0
for i in "${!offboard_pids[@]}"; do
  set +e
  wait "${offboard_pids[i]}"
  rc=$?
  set -e
  n="${SC_AGENT_IDS[i]}"
  if ((rc == 0)); then
    ok "agent $n: route complete, landed"
  else
    warn "agent $n: offboard node exited with $rc (see offboard_$n.log)"
    flight_rc=1
  fi
done

# 7. Stop recording and everything else, then score.
demo_cleanup
log "evaluating against ground truth"
set +e
python3 "$REPO_ROOT/scripts/eval_phase1.py" "$logdir" --check | tee "$logdir/results.txt"
eval_rc=${PIPESTATUS[0]}
set -e

if ((flight_rc != 0)); then die "an agent did not complete its route"; fi
if ((eval_rc != 0)); then die "fused track missed its acceptance thresholds"; fi
ok "Phase 1 demo complete: $logdir"
