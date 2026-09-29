#!/usr/bin/env bash
# Start the simulated world and every agent in a scenario:
# Micro XRCE-DDS agent + Gazebo (one world) + one PX4 SITL instance per agent,
# plus the simulation-side ROS bridges (/clock, /sim/ground_truth and, with a
# ground device in the scenario, its camera).
#
#   scripts/sim.sh                         # headless (default)
#   scripts/sim.sh --gui                   # with the Gazebo GUI
#   scripts/sim.sh --scenario sim/scenarios/two_agent_wall.yaml
#
# Agent n runs as PX4 instance n: its uXRCE-DDS topics are /px4_n/fmu/...
# (PX4 namespaces instance n itself), MAV_SYS_ID is n+1, and all instances
# share one XRCE agent on UDP 8888. Stays in the foreground; Ctrl+C (or
# SIGTERM) stops everything it started.
#
# Refuses to start if a Gazebo, PX4 or XRCE agent is already running: a stale
# simulator silently shares topics with the new one. scripts/demo_phase1.sh
# clears stale processes itself before calling this.
set -euo pipefail
source "$(dirname "$0")/env.sh"

gui=0
scenario="$REPO_ROOT/sim/scenarios/two_agent_wall.yaml"
logdir="${COOP_LOG_DIR:-/tmp/coop-sim}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui) gui=1 ;;
    --headless) gui=0 ;;
    --scenario) scenario="$(realpath "$2")"; shift ;;
    --logdir) logdir="$2"; shift ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done
mkdir -p "$logdir"

px4_build="$PX4_DIR/build/px4_sitl_default"
[[ -x "$px4_build/bin/px4" ]] || die "PX4 SITL not built at $px4_build (run scripts/check_env.sh)"
command -v MicroXRCEAgent >/dev/null || die "MicroXRCEAgent not found (run scripts/check_env.sh)"
[[ -x "$(ros2 pkg prefix synthetic_detector 2>/dev/null)/lib/synthetic_detector/ground_truth_bridge" ]] \
  || die "workspace not built: scripts/setup/build_workspace.sh"

# Anchored patterns: a Gazebo command line starts with "gz sim"; an unanchored
# pattern would also match any shell whose command line merely mentions it.
stale="$(pgrep -f '^gz sim' || true) $(pgrep -x px4 || true) $(pgrep -x MicroXRCEAgent || true)"
if [[ -n "${stale// /}" ]]; then
  die "simulator processes already running (pids:$(echo $stale | tr '\n' ' ')). Stop them, or run scripts/demo_phase1.sh, which clears them."
fi

eval "$(python3 "$REPO_ROOT/scripts/lib/scenario.py" env "$scenario")"
params_dir="$logdir/sim_params"
python3 "$REPO_ROOT/scripts/lib/scenario.py" params "$scenario" "$params_dir" >/dev/null

# Project PX4 parameters: shared ones first, then SITL-only relaxations.
project_params=()
for f in common.params sitl_only.params; do
  while read -r name value _; do
    [[ -z "$name" || "$name" == \#* ]] && continue
    project_params+=("$name=$value")
  done < "$REPO_ROOT/firmware/params/$f"
done

pids=()
cleanup() {
  trap - EXIT INT TERM
  local pid
  for ((i = ${#pids[@]} - 1; i >= 0; i--)); do kill "${pids[i]}" 2>/dev/null || true; done
  for _ in $(seq 1 20); do
    local alive=0
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
    ((alive)) || break
    sleep 0.25
  done
  for pid in "${pids[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
  # A Gazebo server was seen ignoring SIGTERM (quad-autonomy-sim): make sure.
  pkill -KILL -f '^gz sim' 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

log "starting MicroXRCEAgent on udp4 :8888"
MicroXRCEAgent udp4 -p 8888 >"$logdir/xrce_agent.log" 2>&1 &
pids+=($!)

# Gazebo resource paths: PX4's models (the agents), then ours (world, entity).
set +u
# shellcheck disable=SC1091
source "$px4_build/rootfs/gz_env.sh"
set -u
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/sim/models:$REPO_ROOT/sim/worlds:$GZ_SIM_RESOURCE_PATH"

log "starting Gazebo: $SC_WORLD (gui=$gui)"
gz sim --verbose=1 -r -s "$SC_WORLD_FILE" >"$logdir/gz_server.log" 2>&1 &
pids+=($!)
gz_pid=$!
if ((gui)); then
  gz sim -g >"$logdir/gz_gui.log" 2>&1 &
  pids+=($!)
fi
for _ in $(seq 1 120); do
  kill -0 "$gz_pid" 2>/dev/null || die "Gazebo exited while starting (see $logdir/gz_server.log)"
  gz service -i --service "/world/$SC_WORLD/scene/info" 2>/dev/null | grep -q "Service providers" && break
  sleep 0.5
done
ok "Gazebo world $SC_WORLD is up"

# Simulation-side ROS bridges. Executables are run directly, not through
# `ros2 run`, whose wrapper process can leave the node orphaned when killed.
# One bridge process: /clock always, the device camera when the scenario has
# a device (/clock stays the first argument: demo_common.sh matches on it).
bridge_topics=("/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")
if ((SC_DEVICE)); then
  bridge_topics+=("$SC_DEVICE_IMAGE@sensor_msgs/msg/Image[gz.msgs.Image"
    "$SC_DEVICE_INFO@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo")
fi
"$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge" \
  "${bridge_topics[@]}" >"$logdir/clock_bridge.log" 2>&1 &
pids+=($!)
"$(ros2 pkg prefix synthetic_detector)/lib/synthetic_detector/ground_truth_bridge" \
  --ros-args --params-file "$params_dir/ground_truth.yaml" >"$logdir/ground_truth.log" 2>&1 &
pids+=($!)

apply_params_post_boot() {  # <instance>
  local n="$1" bin="$px4_build/bin"
  # rcS applies PX4_PARAM_* before the airframe scripts, and a later
  # `param set-default` can silently override a value equal to its compiled
  # default (quad-autonomy-sim, NAV_DLL_ACT). So set them again after boot.
  # The logger is the last module rcS starts: running == boot done.
  for _ in $(seq 1 240); do
    "$bin/px4-logger" --instance "$n" status >/dev/null 2>&1 && break
    sleep 0.5
  done
  local kv
  for kv in "${project_params[@]}"; do
    "$bin/px4-param" --instance "$n" set "${kv%%=*}" "${kv#*=}" >/dev/null 2>&1 \
      || warn "agent $n: could not set ${kv%%=*}"
  done
  log "agent $n: applied ${#project_params[@]} project parameters after boot"
}

state_root="${COOP_SITL_STATE_DIR:-$HOME/.local/state/coop-perception-sim/sitl}"
px4_pids=()
for n in "${SC_AGENT_IDS[@]}"; do
  pose_var="SC_SPAWN_$n"
  # One working directory per agent: PX4 keeps parameters, dataman and logs there.
  workdir="$state_root/agent_$n"
  mkdir -p "$workdir"
  log "starting PX4 instance $n ($SC_PX4_MODEL at ${!pose_var}), topics /px4_$n/fmu/..."
  (
    cd "$workdir"
    for kv in "${project_params[@]}"; do export "PX4_PARAM_${kv%%=*}=${kv#*=}"; done
    export PX4_GZ_STANDALONE=1 PX4_SIM_MODEL="gz_$SC_PX4_MODEL" PX4_GZ_WORLD="$SC_WORLD" \
      PX4_GZ_MODEL_POSE="${!pose_var}"
    # stdin from /dev/null + -d: daemon mode, no pxh> console.
    exec "$px4_build/bin/px4" -i "$n" -d "$px4_build/etc" </dev/null
  ) >"$logdir/px4_$n.log" 2>&1 &
  pids+=($!)
  px4_pids+=($!)
  apply_params_post_boot "$n" >>"$logdir/px4_$n.log" 2>&1 &
  pids+=($!)
done

ok "simulation starting; logs in $logdir. Ctrl+C to stop."
# If Gazebo or any PX4 instance dies, stop everything.
wait -n "$gz_pid" "${px4_pids[@]}" 2>/dev/null || true
warn "a simulator process exited; shutting down"
