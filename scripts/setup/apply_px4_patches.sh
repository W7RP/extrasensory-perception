#!/usr/bin/env bash
# Apply firmware/px4_patches to the PX4 tree and rebuild what they touch.
# Idempotent: a patch that already reverse-applies is skipped, and nothing is
# rebuilt unless something was applied. No sudo.
#
#   0001, 0002  PX4 itself (quad-autonomy-sim's patches; git apply in $PX4_DIR)
#   0003        PX4-OpticalFlow, the simulated flow sensor's library, which
#               PX4's build fetches into build/px4_sitl_default/OpticalFlow.
#               Needed for more than one flow vehicle per Gazebo server; see
#               firmware/README.md.
set -euo pipefail
source "$(dirname "$0")/common.sh"

build="$PX4_DIR/build/px4_sitl_default"
[[ -d "$PX4_DIR/.git" ]] || die "PX4 not found at $PX4_DIR"

px4_changed=0
for patch in "$REPO_ROOT"/firmware/px4_patches/000[12]-*.patch; do
  name="$(basename "$patch")"
  if git -C "$PX4_DIR" apply --reverse --check "$patch" 2>/dev/null; then
    ok "already applied: $name"
  else
    git -C "$PX4_DIR" apply "$patch" || die "does not apply: $name (PX4 version changed?)"
    ok "applied: $name"
    px4_changed=1
  fi
done
if ((px4_changed)) || [[ ! -x "$build/bin/px4" ]]; then
  log "building PX4 SITL"
  make -C "$PX4_DIR" px4_sitl -j"${JOBS:-$(nproc)}"
fi

flow_src="$build/OpticalFlow/src/OpticalFlow"
[[ -d "$flow_src" ]] || die "PX4-OpticalFlow not fetched yet at $flow_src: build PX4 SITL first"
patch="$REPO_ROOT/firmware/px4_patches/0003-px4-opticalflow-per-instance-state.patch"
if patch -d "$flow_src" -p1 -R --dry-run --silent <"$patch" >/dev/null 2>&1; then
  ok "already applied: $(basename "$patch")"
else
  patch -d "$flow_src" -p1 --forward <"$patch" || die "does not apply: $(basename "$patch")"
  ok "applied: $(basename "$patch")"
  # The patch changes a class layout: rebuild the library AND the Gazebo
  # plugin that instantiates it (it is compiled against the header).
  cmake --build "$build/OpticalFlow/src/OpticalFlow-build" --target install
  ninja -C "$build" OpticalFlowSystem
  ok "rebuilt libOpticalFlow and the OpticalFlowSystem Gazebo plugin"
fi
