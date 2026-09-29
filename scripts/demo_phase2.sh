#!/usr/bin/env bash
# Phase 2 demo, end to end: the ground device walks west of the building
# while the entity walks behind it and the two agents fly their routes. The
# device fuses the agents' detections and draws the see-through overlay on its
# own camera image. Then everything is scored against ground truth.
#
#   scripts/demo_phase2.sh              # headless (the default)
#   scripts/demo_phase2.sh --gui        # Gazebo GUI, RViz, and the overlay image live
#   POSE_SOURCE=ground_truth scripts/demo_phase2.sh   # detectors get perfect navigation
#   RECORD_IMU=1 scripts/demo_phase2.sh      # also record each agent's raw PX4 IMU stream
#
# Same lifecycle as demo_phase1.sh: clears stale processes, polls for every
# stage, tears everything down on exit, including Ctrl-C.
#
# Output: logs/phase2_<timestamp>/ with every process's log, the parameter
# files, the rosbag (coop_bag/), overlay.mp4 (the device's annotated view),
# frames/ (a still every 2 s), metrics.json, results.txt and phase2.png.
# Exit code 0 only if both agents completed their routes AND the overlay met
# the acceptance thresholds in scripts/eval_phase2.py.
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
    -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done

demo_logdir phase2
logdir="$COOP_LOG_DIR"
trap demo_cleanup EXIT
trap 'exit 130' INT TERM
log "logs -> $logdir"

demo_clear_stale

eval "$(python3 "$REPO_ROOT/scripts/lib/scenario.py" env "$scenario")"
((SC_DEVICE)) || die "scenario has no device section: $scenario"
params="$logdir/params"
python3 "$REPO_ROOT/scripts/lib/scenario.py" params "$scenario" "$params" >/dev/null
if [[ -n "${POSE_SOURCE:-}" ]]; then
  sed -i "s/^    pose_source: .*/    pose_source: $POSE_SOURCE/" "$params"/detector_*.yaml
  warn "detectors use pose_source=$POSE_SOURCE"
fi

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
demo_wait_for_message "$SC_DEVICE_INFO" sensor_msgs/msg/CameraInfo 60
ok "ground truth, and the device camera"

# Agents (ESKF + detector), the device (fusion + overlay), single-agent baselines.
agents_start "$params" "$logdir"
mkdir -p "$logdir/frames"
device_start "$params" "$logdir" -p video_path:="$logdir/overlay.mp4" \
  -p snapshot_dir:="$logdir/frames"
baselines_start "$params" "$logdir"
stack_wait_ready /device/tracks
demo_wait_for_message /device/overlay/image sensor_msgs/msg/Image 30
ok "device overlay up"
if ((gui)); then
  "$(ros2 pkg prefix rviz2)/lib/rviz2/rviz2" -d "$REPO_ROOT/sim/rviz/phase2.rviz" \
    --ros-args -p use_sim_time:=true >"$logdir/rviz.log" 2>&1 &
  demo_pids+=($!)
fi

extra_topics=()
if [[ -n "${RECORD_IMU:-}" ]]; then
  for n in "${SC_AGENT_IDS[@]}"; do extra_topics+=("/px4_$n/fmu/out/sensor_combined"); done
fi
# shellcheck disable=SC2046
demo_start_bag "$logdir/coop_bag" $(stack_record_topics /device/tracks) \
  /device/overlay/truth "${extra_topics[@]}"
demo_wait_for_message /device/tracks coop_msgs/msg/TrackArray 10

# The mission: entity and device start moving, both agents fly their routes.
python3 "$REPO_ROOT/scripts/scene_mover.py" --scenario "$scenario" >"$logdir/scene_mover.log" 2>&1 &
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

demo_cleanup   # SIGTERM lets the overlay node close overlay.mp4 cleanly
log "evaluating against ground truth"
set +e
python3 "$REPO_ROOT/scripts/eval_phase2.py" "$logdir" --check | tee "$logdir/results.txt"
eval_rc=${PIPESTATUS[0]}
set -e

if ((flight_rc != 0)); then die "an agent did not complete its route"; fi
if ((eval_rc != 0)); then die "overlay missed its acceptance thresholds"; fi
ok "Phase 2 demo complete: $logdir"
