# Source this (don't execute it) in every shell used for the project:
#   source scripts/env.sh
# Sets up ROS 2 Humble, this repo's workspace overlay (if built), and the
# locally installed Micro XRCE-DDS agent.

_coop_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=setup/common.sh
source "$_coop_root/scripts/setup/common.sh"

# ROS setup scripts reference unset variables; relax nounset while sourcing them.
_coop_nounset=0
if [[ $- == *u* ]]; then _coop_nounset=1; set +u; fi
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[env] /opt/ros/humble not found (see docs/running.md, Setup)" >&2
fi
if [[ -f "$_coop_root/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$_coop_root/ros2_ws/install/setup.bash"
fi
if ((_coop_nounset)); then set -u; fi

case ":$PATH:" in *":$XRCE_AGENT_PREFIX/bin:"*) ;; *) export PATH="$XRCE_AGENT_PREFIX/bin:$PATH" ;; esac
case ":${LD_LIBRARY_PATH:-}:" in
  *":$XRCE_AGENT_PREFIX/lib:"*) ;;
  *) export LD_LIBRARY_PATH="$XRCE_AGENT_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
esac

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# Gazebo transport stays on loopback: no multicast discovery onto the LAN.
export GZ_IP=127.0.0.1
unset _coop_root _coop_nounset
