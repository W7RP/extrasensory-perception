# Shared plumbing for scripts/{sim,stack,fly,demo_phase1}.sh. Sourced, not
# executed. Expects scripts/env.sh to have been sourced (log/ok/warn/die,
# REPO_ROOT). Adapted from quad-autonomy-sim's demo_common.sh, where each rule
# below was learned from a failure.

demo_pids=()

# Every process a previous run could have left behind. A stale Gazebo server
# makes PX4 attach to the wrong world, and a stale node keeps publishing into
# the new run's topics; both were seen in quad-autonomy-sim. Patterns are exact
# process names (pgrep -x) or anchored command lines (^...): an unanchored
# pattern would also match any shell that merely mentions the name. Project
# nodes are matched by their install path, because Linux truncates process
# names to 15 characters (synthetic_detector would never match -x).
_stale_exact=(px4 MicroXRCEAgent)
_stale_anchored=('^gz sim' '^[^ ]*parameter_bridge /clock@' '^python3 [^ ]*entity_mover\.py'
  '^[^ ]*/lib/(agent_estimation|agent_offboard|synthetic_detector|track_fusion)/'
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 bag record .*coop_bag')

_stale_pids() {
  local name pat
  for name in "${_stale_exact[@]}"; do pgrep -x "$name" || true; done
  for pat in "${_stale_anchored[@]}"; do pgrep -f "$pat" || true; done
}

# demo_clear_stale: stop every leftover simulator/project process, TERM then KILL.
demo_clear_stale() {
  local pids
  pids="$(_stale_pids | sort -u | tr '\n' ' ')"
  if [[ -z "${pids// /}" ]]; then
    ok "no stale simulator processes"
    return 0
  fi
  warn "stopping stale processes: $(ps -o pid=,comm= -p ${pids// /,} | tr -s ' \n' ' ')"
  # shellcheck disable=SC2086
  kill $pids 2>/dev/null || true
  for _ in $(seq 1 50); do
    [[ -z "$(_stale_pids | tr -d '\n')" ]] && break
    sleep 0.1
  done
  pids="$(_stale_pids | sort -u | tr '\n' ' ')"
  if [[ -n "${pids// /}" ]]; then
    # Gazebo servers were seen ignoring SIGTERM.
    # shellcheck disable=SC2086
    kill -KILL $pids 2>/dev/null || true
    sleep 0.5
  fi
  [[ -z "$(_stale_pids | tr -d '\n')" ]] || die "could not clear stale processes: $(_stale_pids | tr '\n' ' ')"
  ok "stale processes cleared"
}

demo_cleanup() {
  demo_stop_bag 20
  # SIGTERM, not SIGINT: background jobs of a non-interactive shell ignore
  # SIGINT. Reverse start order: mission and perception nodes first, the
  # simulator (sim.sh, which stops its own children) last.
  local i pid
  for ((i = ${#demo_pids[@]} - 1; i >= 0; i--)); do
    kill "${demo_pids[i]}" 2>/dev/null || true
  done
  for _ in $(seq 1 150); do
    local alive=0
    for pid in "${demo_pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
    ((alive)) || break
    sleep 0.1
  done
  for pid in "${demo_pids[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
  demo_pids=()
  # Nothing may outlive a run (see demo_clear_stale).
  pkill -KILL -f '^gz sim' 2>/dev/null || true
  wait 2>/dev/null || true
}

# demo_logdir <name>: creates logs/<name>_<timestamp>, sets and exports
# COOP_LOG_DIR (not echoed: a $(...) subshell would lose the export).
demo_logdir() {
  COOP_LOG_DIR="$REPO_ROOT/logs/${1}_$(date +%Y%m%d_%H%M%S)"
  export COOP_LOG_DIR
  mkdir -p "$COOP_LOG_DIR"
}

# demo_start_sim <logdir> [sim.sh args...]   (background)
demo_start_sim() {
  local dir="$1"
  shift
  "$REPO_ROOT/scripts/sim.sh" --logdir "$dir" "$@" </dev/null >"$dir/sim.log" 2>&1 &
  demo_pids+=($!)
  demo_sim_pid=$!
}

# demo_start_node <logfile> <package> <executable> [args...]   (background)
# Runs the executable directly rather than through `ros2 run`, whose wrapper
# process can leave the node orphaned when killed.
demo_start_node() {
  local logfile="$1" pkg="$2" exe="$3"
  shift 3
  "$(ros2 pkg prefix "$pkg")/lib/$pkg/$exe" "$@" >"$logfile" 2>&1 &
  demo_pids+=($!)
}

# demo_wait_for_message <topic> <type> <timeout_s>
# Polls for an actual message, not just the topic name (`ros2 topic list` can
# be answered from a daemon that remembers an earlier session). --no-daemon
# everywhere: the shared daemon was seen stuck, silently failing every call.
# Best-effort QoS matches both best-effort (PX4) and reliable publishers.
demo_wait_for_message() {
  local topic="$1" type="$2" timeout_s="$3" start=$SECONDS
  while ((SECONDS - start < timeout_s)); do
    if [[ -n "${demo_sim_pid:-}" ]] && ! kill -0 "$demo_sim_pid" 2>/dev/null; then
      die "simulator exited early (see $COOP_LOG_DIR/sim.log)"
    fi
    if timeout 5 ros2 topic echo --no-daemon --once --qos-reliability best_effort \
        "$topic" "$type" >/dev/null 2>&1; then
      return 0
    fi
  done
  die "no message on $topic after ${timeout_s} s (logs: $COOP_LOG_DIR)"
}

# demo_start_bag <dir> <topics...>   (background, own process group)
# `ros2 bag record` finalises the bag's metadata on SIGINT, so it runs under
# setsid and demo_stop_bag interrupts the whole group.
demo_start_bag() {
  local dir="$1" pidfile
  shift
  pidfile="$(mktemp)"
  # setsid forks when its caller leads a process group, so $! is not the new
  # group's leader; the shell inside records its own PID (= PGID) first.
  setsid bash -c 'echo $$ >"$0"; exec ros2 bag record --storage sqlite3 -o "$1" "${@:2}"' \
    "$pidfile" "$dir" "$@" >"$(dirname "$dir")/bag.log" 2>&1 &
  for _ in $(seq 1 50); do [[ -s "$pidfile" ]] && break; sleep 0.1; done
  demo_bag_pgid="$(cat "$pidfile")"
  rm -f -- "$pidfile"
}

demo_stop_bag() {
  local timeout_s="${1:-20}" start=$SECONDS
  [[ -n "${demo_bag_pgid:-}" ]] || return 0
  kill -INT -- "-$demo_bag_pgid" 2>/dev/null || true
  while kill -0 -- "-$demo_bag_pgid" 2>/dev/null && ((SECONDS - start < timeout_s)); do
    sleep 0.2
  done
  kill -KILL -- "-$demo_bag_pgid" 2>/dev/null || true
  demo_bag_pgid=""
}

# ------------------------------------------------------------ the stack

# stack_start <params_dir> <logdir> [pose_source]
# Per agent: ESKF (navigation) and detector. Then the fused tracker plus one
# single-agent baseline tracker per agent, all identical but for their inputs.
stack_start() {
  local params="$1" dir="$2" n
  for n in "${SC_AGENT_IDS[@]}"; do
    demo_start_node "$dir/eskf_$n.log" agent_estimation eskf_node --ros-args \
      -r __ns:="/agent_$n" \
      --params-file "$(ros2 pkg prefix agent_estimation)/share/agent_estimation/config/eskf.yaml" \
      --params-file "$params/eskf_$n.yaml"
    demo_start_node "$dir/detector_$n.log" synthetic_detector synthetic_detector --ros-args \
      -r __ns:="/agent_$n" --params-file "$params/detector_$n.yaml"
  done
  demo_start_node "$dir/fusion.log" track_fusion fusion_node --ros-args \
    -r __ns:=/fusion --params-file "$params/fusion.yaml"
  for n in "${SC_AGENT_IDS[@]}"; do
    demo_start_node "$dir/baseline_$n.log" track_fusion fusion_node --ros-args \
      -r __ns:="/baseline_agent_$n" --params-file "$params/baseline_$n.yaml"
  done
}

# stack_wait_ready: every agent's ESKF aligned and detector publishing, fusion up.
stack_wait_ready() {
  local n
  for n in "${SC_AGENT_IDS[@]}"; do
    demo_wait_for_message "/agent_$n/eskf/odometry" nav_msgs/msg/Odometry 60
    ok "agent $n: ESKF aligned"
    demo_wait_for_message "/agent_$n/detections" coop_msgs/msg/DetectionArray 30
  done
  demo_wait_for_message /fusion/tracks coop_msgs/msg/TrackArray 30
  ok "perception stack up"
}

# The topics the evaluation reads.
stack_record_topics() {
  local n
  echo /sim/ground_truth /fusion/tracks /diagnostics
  for n in "${SC_AGENT_IDS[@]}"; do
    echo "/agent_$n/detections" "/agent_$n/visibility_truth" "/agent_$n/eskf/odometry" \
      "/baseline_agent_$n/tracks"
  done
}
