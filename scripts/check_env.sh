#!/usr/bin/env bash
# Verify the toolchain this project needs is installed, at the expected
# versions. Read-only: installs nothing, changes nothing, never uses sudo.
# The toolchain comes from quad-autonomy-sim's setup scripts, plus this repo's
# scripts/setup/apply_px4_patches.sh (see docs/running.md). Exit code = number
# of failed checks.
set -uo pipefail
source "$(dirname "$0")/env.sh"

fails=0
check() {  # <description> <command...>: the command prints the found value
  local what="$1"; shift
  local out
  if out="$("$@" 2>/dev/null)" && [[ -n "$out" ]]; then
    ok "$what: $out"
  else
    warn "$what: MISSING or wrong${out:+ ($out)}"
    fails=$((fails + 1))
  fi
}
expect() {  # <description> <expected> <command...>
  local what="$1" want="$2"; shift 2
  local out
  out="$("$@" 2>/dev/null)"
  if [[ "$out" == "$want" ]]; then
    ok "$what: $out"
  else
    warn "$what: found '${out:-nothing}', expected '$want'"
    fails=$((fails + 1))
  fi
}

expect "Ubuntu" "22.04" bash -c '. /etc/os-release; echo "$VERSION_ID"'
expect "ROS 2 distro" "$ROS_DISTRO_EXPECTED" bash -c 'echo "${ROS_DISTRO:-}"'
expect "Gazebo Sim" "$GZ_SIM_VERSION_EXPECTED" bash -c 'gz sim --versions | head -1 | tr -d " "'
expect "PX4 source" "$PX4_VERSION" git -C "$PX4_DIR" describe --tags --exact-match
check "PX4 SITL binary" bash -c "test -x '$PX4_DIR/build/px4_sitl_default/bin/px4' && echo '$PX4_DIR/build/px4_sitl_default/bin/px4'"
for patch in "$REPO_ROOT"/firmware/px4_patches/000[12]-*.patch; do
  check "PX4 patch $(basename "$patch")" bash -c \
    "git -C '$PX4_DIR' apply --reverse --check '$patch' && echo applied"
done
flow_patch="$REPO_ROOT/firmware/px4_patches/0003-px4-opticalflow-per-instance-state.patch"
check "PX4-OpticalFlow patch $(basename "$flow_patch")" bash -c \
  "patch -d '$PX4_DIR/build/px4_sitl_default/OpticalFlow/src/OpticalFlow' -p1 -R --dry-run --silent <'$flow_patch' >/dev/null && echo applied"
check "flow plugin rebuilt after it" bash -c \
  "test '$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins/libOpticalFlowSystem.so' -nt '$PX4_DIR/build/px4_sitl_default/OpticalFlow/install/include/flow_opencv.hpp' && echo yes"
check "PX4 build has the patched topic list" bash -c \
  "grep -q sensor_optical_flow '$PX4_DIR/build/px4_sitl_default/src/modules/uxrce_dds_client/dds_topics.h' && echo yes"
check "MicroXRCEAgent" bash -c "command -v MicroXRCEAgent"
check "ros_gz_bridge (Harmonic build)" dpkg-query -W -f='${Version}' ros-humble-ros-gzharmonic-bridge
check "gz-transport Python bindings" python3 -c "import gz.transport13; print('gz.transport13')"
check "rosbag2 Python API" python3 -c "import rosbag2_py; print('rosbag2_py')"
check "matplotlib" python3 -c "import matplotlib; print(matplotlib.__version__)"
check "libsdformat14" dpkg-query -W -f='${Version}' libsdformat14-dev
check "px4_msgs in the workspace" bash -c \
  "git -C '$REPO_ROOT/ros2_ws/src/external/px4_msgs' rev-parse HEAD"

if ((fails)); then
  warn "$fails check(s) failed. See docs/running.md (Setup)."
else
  ok "environment matches docs/environment.md"
fi
exit "$fails"
