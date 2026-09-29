#!/usr/bin/env bash
# Phase 4: one high overwatch agent instead of a swarm. A single GPS agent at
# 100 m with a wide 4K search camera and a gimbal-mounted zoom camera (real
# sensor specs, synthetic_detector/imaging.hpp) covers the device's
# surroundings on a larger map, and the zoom looks closer at one track at a
# time; the device shows its video as a picture-in-picture.
#
#   scripts/demo_phase4.sh --play               # you are the device (as in Phase 3)
#   scripts/demo_phase4.sh                      # scored: scripted device, headless
#   scripts/demo_phase4.sh --low                # same map, Phase 3's two low agents instead
#   scripts/demo_phase4.sh --altitude 140       # the high agent at another altitude
#   scripts/demo_phase4.sh --nav-bias 1.5,2.5   # agents' GPS error (1-sigma horizontal,vertical [m])
#   scripts/demo_phase4.sh --thermal            # add a thermal camera (sees through some canopy)
#   scripts/demo_phase4.sh --gui ...            # also the Gazebo GUI
#   DURATION=180 scripts/demo_phase4.sh         # scored session length [s of simulation]
#
# The map (seed 11, 100 x 100 m) is generated on first use, in both profiles,
# from the same layout. Everything else (lifecycle, recording, scoring) is
# demo_phase3.sh's; scoring is eval_phase4.py, from the moment the agents are
# on station. scripts/compare_phase4.sh runs the whole comparison.
#
# Output: logs/phase4_<timestamp>/, as in Phase 3, with metrics.json,
# results.txt and phase4.png.
set -euo pipefail
source "$(dirname "$0")/env.sh"

profile=high
args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --low) profile=low ;;
    --altitude) export ALTITUDE="$2"; shift ;;
    --nav-bias) export NAV_BIAS="$2"; shift ;;
    --thermal) export THERMAL=1 ;;
    --play|--gui|--headless) args+=("$1") ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done

# The Phase 4 map, both profiles, one layout (generate_map.py documents the options).
map_args=(--seed 11 --size 100 --buildings 18 --walls 12 --containers 12 --crates 25 --trees 45
  --entities 10 --near-entities 6 --interest-radius 30)
suffix() { if [[ $1 == high ]]; then echo _high; fi; }
for p in low high; do
  name="map_11$(suffix "$p")"
  if [[ ! -f "$REPO_ROOT/sim/scenarios/$name.yaml" ]]; then
    log "generating the Phase 4 map ($p profile)"
    python3 "$REPO_ROOT/sim/tools/generate_map.py" "${map_args[@]}" --profile "$p"
  fi
done
map="map_11$(suffix "$profile")"
DEMO_TAG="phase4_$profile" DEMO_EVAL=eval_phase4.py DURATION="${DURATION:-150}" \
  exec "$REPO_ROOT/scripts/demo_phase3.sh" --map "$map" "${args[@]}"
