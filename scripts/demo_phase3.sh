#!/usr/bin/env bash
# Phase 3: the playable scene. A generated map, several entities walking
# their loops among buildings, walls, containers and trees, and two GPS agents
# that position themselves around the ground device (the overwatch planner)
# so that, between them, they see what it cannot.
#
#   scripts/demo_phase3.sh --play            # you are the device: WASD + mouse in the game window
#   scripts/demo_phase3.sh                   # scored: the device walks a scripted loop, headless
#   scripts/demo_phase3.sh --seed 12 --play  # another map (generated on first use)
#   scripts/demo_phase3.sh --gui ...         # also the Gazebo GUI
#   DURATION=180 scripts/demo_phase3.sh      # scored session length [s of simulation]
#   GAME_INPUT="x:20,w:8,q:2,wR:6" scripts/demo_phase3.sh --play   # scripted keys (keys:seconds)
#
# Same lifecycle rules as the other demos: clears stale processes, polls for
# every stage, tears everything down on exit (Ctrl-C too). A played session
# ends when you press Esc (or close the window): the agents land, and the
# session is scored like any other.
#
# Output: logs/phase3_<timestamp>/ with every process's log, the scenario and
# parameter files used, the rosbag (coop_bag/), overlay.mp4 (the device's
# annotated view), frames/, metrics.json, results.txt and phase3.png.
# Scored mode exits 0 only if the acceptance thresholds in eval_phase3.py hold.
set -euo pipefail
source "$(dirname "$0")/env.sh"
source "$REPO_ROOT/scripts/lib/demo_common.sh"

gui=0
play=0
seed=7
while [[ $# -gt 0 ]]; do
  case "$1" in
    --play) play=1 ;;
    --gui) gui=1 ;;
    --headless) gui=0 ;;
    --seed) seed="$2"; shift ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done

demo_logdir phase3
logdir="$COOP_LOG_DIR"
trap demo_cleanup EXIT
trap 'exit 130' INT TERM
log "logs -> $logdir"
demo_clear_stale

# The map: generated once per seed, then reused (it is deterministic).
if [[ ! -f "$REPO_ROOT/sim/scenarios/map_$seed.yaml" ]]; then
  log "generating map $seed"
  python3 "$REPO_ROOT/sim/tools/generate_map.py" --seed "$seed"
fi
# This session's copy of the scenario: driven or scripted device, session length.
scenario="$logdir/scenario.yaml"
python3 - "$REPO_ROOT/sim/scenarios/map_$seed.yaml" "$scenario" "$play" "${DURATION:-150}" <<'EOF'
import sys, yaml
src, dst, play, duration = sys.argv[1], sys.argv[2], sys.argv[3] == "1", float(sys.argv[4])
sc = yaml.safe_load(open(src))
sc["world_file"] = str((__import__("pathlib").Path(src).parents[2] / sc["world_file"]).resolve())
sc["device"]["control"] = "teleop" if play else "scripted"
sc["planner"]["duration_s"] = 0.0 if play else duration
yaml.safe_dump(sc, open(dst, "w"), sort_keys=False)
EOF
eval "$(python3 "$REPO_ROOT/scripts/lib/scenario.py" env "$scenario")"
params="$logdir/params"
python3 "$REPO_ROOT/scripts/lib/scenario.py" params "$scenario" "$params" >/dev/null

sim_args=(--scenario "$scenario")
((gui)) && sim_args+=(--gui)
demo_start_sim "$logdir" "${sim_args[@]}"
log "waiting for the simulator (map $seed)"
demo_wait_for_message /clock rosgraph_msgs/msg/Clock 60
for n in "${SC_AGENT_IDS[@]}"; do
  demo_wait_for_message "/px4_$n/fmu/out/vehicle_status_v1" px4_msgs/msg/VehicleStatus 120
  ok "agent $n: PX4 <-> ROS 2 bridge up"
done
demo_wait_for_message /sim/ground_truth tf2_msgs/msg/TFMessage 60
demo_wait_for_message "$SC_DEVICE_INFO" sensor_msgs/msg/CameraInfo 60
ok "ground truth, and the device camera"

agents_start "$params" "$logdir"
mkdir -p "$logdir/frames"
truth_ghost=true
((play)) && truth_ghost=false   # a played session shows only what the device knows
overlay_hud=true
((play)) && overlay_hud=false   # the game window draws its own HUD
device_start "$params" "$logdir" -p video_path:="$logdir/overlay.mp4" \
  -p snapshot_dir:="$logdir/frames" -p draw_truth:="$truth_ghost" -p draw_hud:="$overlay_hud"
device_body_start "$params" "$logdir"
baselines_start "$params" "$logdir"
stack_wait_ready /device/tracks
demo_wait_for_message /device/overlay/image sensor_msgs/msg/Image 30
demo_wait_for_message /device/status coop_msgs/msg/DeviceStatus 20
ok "device up"
if ((play)); then
  game_args=(-p video_path:="$logdir/game.mp4" -p snapshot_dir:="$logdir/frames")
  [[ -n "${GAME_INPUT:-}" ]] && game_args+=(-p demo_input:="$GAME_INPUT")
  demo_start_node "$logdir/game_view.log" device_view game_view --ros-args \
    -r __ns:=/device --params-file "$params/game_view.yaml" "${game_args[@]}"
  log "game window open: WASD to walk, Q/E or right mouse to turn, Shift to run, Esc to end"
fi

extra_topics=(/device/overlay/truth /device/status)
for n in "${SC_AGENT_IDS[@]}"; do extra_topics+=("/agent_$n/status" "/agent_$n/goal"); done
# shellcheck disable=SC2046
demo_start_bag "$logdir/coop_bag" $(stack_record_topics /device/tracks) "${extra_topics[@]}"
demo_wait_for_message /device/tracks coop_msgs/msg/TrackArray 10

# The session: entities walk (and, in scored mode, the device), the planner
# tasks the agents, the agents fly its goals until it says land.
python3 "$REPO_ROOT/scripts/scene_mover.py" --scenario "$scenario" >"$logdir/scene_mover.log" 2>&1 &
demo_pids+=($!)
planner_start "$params" "$logdir"
offboard_pids=()
for n in "${SC_AGENT_IDS[@]}"; do
  demo_start_node "$logdir/offboard_$n.log" agent_offboard offboard_agent --ros-args \
    -r __ns:="/agent_$n" \
    --params-file "$(ros2 pkg prefix agent_offboard)/share/agent_offboard/config/offboard.yaml" \
    --params-file "$params/offboard_$n.yaml"
  offboard_pids+=("${demo_pids[-1]}")
done
if ((play)); then log "session running: press Esc in the game window to end it"; else
  log "scored session: ${DURATION:-150} s of simulation, then the agents land"; fi
flight_rc=0
for i in "${!offboard_pids[@]}"; do
  set +e
  wait "${offboard_pids[i]}"
  rc=$?
  set -e
  n="${SC_AGENT_IDS[i]}"
  if ((rc == 0)); then ok "agent $n: landed"; else warn "agent $n: offboard node exited with $rc"; flight_rc=1; fi
done

demo_cleanup
log "scoring the session"
set +e
check=(--check)
((play)) && check=()
python3 "$REPO_ROOT/scripts/eval_phase3.py" "$logdir" "${check[@]}" | tee "$logdir/results.txt"
eval_rc=${PIPESTATUS[0]}
set -e
if ((flight_rc != 0)); then die "an agent did not complete its session"; fi
if ((eval_rc != 0)); then die "session missed its acceptance thresholds"; fi
ok "Phase 3 session complete: $logdir"
