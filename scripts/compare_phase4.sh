#!/usr/bin/env bash
# Phase 4's comparison: one high overwatch agent against Phase 3's two low
# agents on the same map and device route, an altitude sweep of the high
# agent, both with GPS-grade navigation error (and the high one with RTK),
# and the high agent with a thermal camera added. Runs the scored sessions
# one after the other (scripts/demo_phase4.sh), then tabulates and plots them
# (scripts/compare_phase4.py).
#
#   scripts/compare_phase4.sh                          # the full set (~1 h)
#   scripts/compare_phase4.sh --altitudes "60 100"     # a shorter sweep
#   scripts/compare_phase4.sh --no-bias                # skip the navigation-error sessions
#
# Every session is scored for the same ~150 s on station (DURATION includes
# the climb, longer for higher altitudes). Output: logs/phase4_compare_<ts>/
# with comparison.md, comparison.json and comparison.png, and the session
# directories listed in sessions.txt. A session that fails its thresholds is
# still compared (and flagged); one that does not complete stops the run.
set -euo pipefail
source "$(dirname "$0")/env.sh"

altitudes="60 100 150 250"
bias=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --altitudes) altitudes="$2"; shift ;;
    --no-bias) bias=0 ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done

out="$REPO_ROOT/logs/phase4_compare_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$out"
: >"$out/sessions.txt"
log "comparison -> $out"

# run <label> <duration_s> [demo_phase4.sh args...]
run() {
  local label="$1" duration="$2"
  shift 2
  log "session: $label"
  set +e
  DURATION="$duration" "$REPO_ROOT/scripts/demo_phase4.sh" "$@" >"$out/$label.log" 2>&1
  local rc=$?
  set -e
  local dir
  dir="$(sed -n 's/.*logs -> \(.*\)$/\1/p' "$out/$label.log" | head -1)"
  [[ -n "$dir" && -f "$dir/metrics.json" ]] || die "session $label did not complete (see $out/$label.log)"
  ((rc == 0)) || warn "session $label missed a threshold (still compared)"
  echo "$label $dir" >>"$out/sessions.txt"
  ok "$label: $dir"
}

# Climb at 3 m/s and fly to the spot, then ~150 s on station.
duration_for() { echo $((150 + $1 / 3 + 20)); }

run low 170 --low
for alt in $altitudes; do
  run "high_$alt" "$(duration_for "$alt")" --altitude "$alt"
done
if ((bias)); then
  run low_gps 170 --low --nav-bias 1.5,2.5
  run high_100_gps "$(duration_for 100)" --altitude 100 --nav-bias 1.5,2.5
  run high_100_rtk "$(duration_for 100)" --altitude 100 --nav-bias 0.03,0.05
fi
run high_100_thermal "$(duration_for 100)" --altitude 100 --thermal
python3 "$REPO_ROOT/scripts/compare_phase4.py" "$out"
