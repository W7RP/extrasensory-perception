#!/usr/bin/env bash
# Build this repo's ROS 2 workspace. No sudo, installs nothing system-wide.
# px4_msgs is fetched into ros2_ws/src/external (git-ignored) at the pinned
# commit if it is not there yet. The rest of the toolchain (ROS 2, PX4 SITL
# with firmware/px4_patches, Gazebo, the XRCE agent) is quad-autonomy-sim's:
# run scripts/check_env.sh first.
set -euo pipefail
source "$(dirname "$0")/common.sh"

ext="$REPO_ROOT/ros2_ws/src/external"
mkdir -p "$ext"
if [[ -d "$ext/px4_msgs/.git" ]]; then
  ok "px4_msgs present ($(git -C "$ext/px4_msgs" rev-parse --short HEAD))"
else
  log "fetching px4_msgs $PX4_MSGS_BRANCH @ ${PX4_MSGS_COMMIT:-head}"
  git clone --branch "$PX4_MSGS_BRANCH" --filter=blob:none https://github.com/PX4/px4_msgs.git "$ext/px4_msgs"
  [[ -n "$PX4_MSGS_COMMIT" ]] && git -C "$ext/px4_msgs" -c advice.detachedHead=false checkout "$PX4_MSGS_COMMIT"
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
cd "$REPO_ROOT/ros2_ws"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --parallel-workers "${JOBS:-$(nproc)}"
ok "workspace built: source scripts/env.sh"
