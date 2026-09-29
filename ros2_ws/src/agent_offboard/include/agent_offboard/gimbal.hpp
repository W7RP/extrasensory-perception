// Pointing logic for a world-stabilised two-axis gimbal (yaw, pitch), as on
// the zoom camera of a high overwatch agent. ROS-free and unit-tested.
//
// Angles are in the world (ENU): yaw is the optical axis' heading from east,
// counter-clockwise; pitch_down is its angle below the horizon (pi/2 = straight
// down). A real gimbal holds these against the airframe's motion with its own
// IMU; here the airframe's attitude never enters.
//
// Each axis turns at most `slew_rate` towards the command, yaw the short way
// round. Near straight down, yaw barely moves the image, so the yaw axis
// follows the command without hurrying the pitch.
#pragma once

#include <array>

namespace agent_offboard
{

struct GimbalAngles
{
  double yaw{0.0};
  double pitch_down{1.5707963267948966};
};

struct GimbalConfig
{
  double slew_rate_rad_s{1.57};     // 90 deg/s, typical of small payload gimbals
  double min_pitch_down_rad{0.0};   // the horizon
  double max_pitch_down_rad{1.5707963267948966};  // straight down
  double hfov_rad{0.1047};          // for the on-target test
  double on_target_fraction{0.2};   // within this fraction of the field of view
};

// The pointing that puts `target` on the optical axis of a camera at `camera`
// (both world ENU [m]), within the pitch limits.
[[nodiscard]] GimbalAngles pointing_to(
  const std::array<double, 3> & camera, const std::array<double, 3> & target,
  const GimbalConfig & cfg) noexcept;

// One control step of dt seconds from `current` towards `command`.
[[nodiscard]] GimbalAngles step(
  const GimbalAngles & current, const GimbalAngles & command, double dt,
  const GimbalConfig & cfg) noexcept;

// Angle between the two pointings' optical axes [rad].
[[nodiscard]] double pointing_error(const GimbalAngles & a, const GimbalAngles & b) noexcept;

[[nodiscard]] bool on_target(
  const GimbalAngles & current, const GimbalAngles & command, const GimbalConfig & cfg) noexcept;

}  // namespace agent_offboard
