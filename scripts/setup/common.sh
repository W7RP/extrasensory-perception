# Shared helpers and pinned versions. Sourced, not executed.
#
# The toolchain is the one quad-autonomy-sim installs and pins (its
# scripts/setup/0*_*.sh). This project reuses it as-is: check_env.sh verifies it,
# build_workspace.sh builds only this repo's ROS 2 workspace.

export PX4_VERSION="${PX4_VERSION:-v1.17.0}"
export PX4_MSGS_BRANCH="${PX4_MSGS_BRANCH:-release/1.17}"
# Exact px4_msgs commit verified working with PX4 v1.17.0 (docs/environment.md).
export PX4_MSGS_COMMIT="${PX4_MSGS_COMMIT-86d8239e962f6939e05c3737784f60c02fa884db}"
export XRCE_AGENT_VERSION="${XRCE_AGENT_VERSION:-v2.4.3}"
export GZ_SIM_VERSION_EXPECTED="8.15.0"
export ROS_DISTRO_EXPECTED="humble"

# Locations: outside the repo (large, their own history, toolchain not project).
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export XRCE_AGENT_PREFIX="${XRCE_AGENT_PREFIX:-$HOME/.local}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export REPO_ROOT

log()  { printf '\033[1;34m[coop]\033[0m %s\n' "$*"; }
ok()   { printf '\033[1;32m[ ok ]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[fail]\033[0m %s\n' "$*" >&2; exit 1; }
