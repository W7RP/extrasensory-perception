#include "agent_offboard/gimbal.hpp"

#include <algorithm>
#include <cmath>

namespace agent_offboard
{

namespace
{
double wrap(double a) noexcept
{
  return std::remainder(a, 2.0 * M_PI);
}

std::array<double, 3> axis(const GimbalAngles & g) noexcept
{
  const double c = std::cos(g.pitch_down);
  return {c * std::cos(g.yaw), c * std::sin(g.yaw), -std::sin(g.pitch_down)};
}
}  // namespace

GimbalAngles pointing_to(
  const std::array<double, 3> & camera, const std::array<double, 3> & target,
  const GimbalConfig & cfg) noexcept
{
  const double dx = target[0] - camera[0];
  const double dy = target[1] - camera[1];
  const double dz = target[2] - camera[2];
  const double horiz = std::hypot(dx, dy);
  GimbalAngles g;
  g.yaw = horiz > 1e-6 ? std::atan2(dy, dx) : 0.0;
  g.pitch_down = std::clamp(std::atan2(-dz, horiz), cfg.min_pitch_down_rad,
      cfg.max_pitch_down_rad);
  return g;
}

GimbalAngles step(
  const GimbalAngles & current, const GimbalAngles & command, double dt,
  const GimbalConfig & cfg) noexcept
{
  const double max_step = cfg.slew_rate_rad_s * std::max(dt, 0.0);
  GimbalAngles out;
  const double dyaw = wrap(command.yaw - current.yaw);
  out.yaw = wrap(current.yaw + std::clamp(dyaw, -max_step, max_step));
  const double target_pitch = std::clamp(command.pitch_down, cfg.min_pitch_down_rad,
      cfg.max_pitch_down_rad);
  out.pitch_down = current.pitch_down +
    std::clamp(target_pitch - current.pitch_down, -max_step, max_step);
  return out;
}

double pointing_error(const GimbalAngles & a, const GimbalAngles & b) noexcept
{
  const auto u = axis(a);
  const auto v = axis(b);
  const double dot = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  return std::acos(std::clamp(dot, -1.0, 1.0));
}

bool on_target(
  const GimbalAngles & current, const GimbalAngles & command, const GimbalConfig & cfg) noexcept
{
  return pointing_error(current, command) <= cfg.on_target_fraction * cfg.hfov_rad;
}

}  // namespace agent_offboard
