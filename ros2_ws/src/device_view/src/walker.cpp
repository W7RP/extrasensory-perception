#include "device_view/walker.hpp"

#include <algorithm>
#include <cmath>

namespace device_view
{

namespace
{
using synthetic_detector::Vec3;

// Does the obstacle's vertical extent overlap the walker's body?
bool at_body_height(double zmin, double zmax, const WalkerConfig & cfg) noexcept
{
  return zmin < cfg.body_top_m && zmax > cfg.body_bottom_m;
}
}  // namespace

bool blocked(
  double x, double y, const WalkerConfig & cfg,
  const synthetic_detector::Occluders & obstacles) noexcept
{
  const double r = cfg.radius_m;
  if (x - r < cfg.xmin || x + r > cfg.xmax || y - r < cfg.ymin || y + r > cfg.ymax) {
    return true;
  }
  for (const auto & b : obstacles.boxes) {
    // Footprint test in the box's frame (boxes stand upright: yaw only).
    const Vec3 p = b.q.conjugate() * (Vec3(x, y, b.center.z()) - b.center);
    const double zmin = b.center.z() - b.half_extents.z();
    const double zmax = b.center.z() + b.half_extents.z();
    if (!at_body_height(zmin, zmax, cfg)) {
      continue;
    }
    const double dx = std::max(std::abs(p.x()) - b.half_extents.x(), 0.0);
    const double dy = std::max(std::abs(p.y()) - b.half_extents.y(), 0.0);
    if (dx * dx + dy * dy < r * r) {
      return true;
    }
  }
  for (const auto & c : obstacles.cylinders) {
    // Upright cylinders only (trunks, posts): a circle footprint.
    const double zmin = c.center.z() - c.half_length;
    const double zmax = c.center.z() + c.half_length;
    if (!at_body_height(zmin, zmax, cfg)) {
      continue;
    }
    if (std::hypot(x - c.center.x(), y - c.center.y()) < c.radius + r) {
      return true;
    }
  }
  for (const auto & s : obstacles.spheres) {
    if (!at_body_height(s.center.z() - s.radius, s.center.z() + s.radius, cfg)) {
      continue;
    }
    if (std::hypot(x - s.center.x(), y - s.center.y()) < s.radius + r) {
      return true;
    }
  }
  return false;
}

WalkerState step(
  const WalkerState & s, const WalkCommand & cmd, double dt, const WalkerConfig & cfg,
  const synthetic_detector::Occluders & obstacles) noexcept
{
  WalkerState out = s;
  out.yaw = std::remainder(s.yaw + std::clamp(cmd.turn, -1.0, 1.0) * cfg.turn_rate_rps * dt,
      2.0 * M_PI);
  double f = std::clamp(cmd.forward, -1.0, 1.0);
  double l = std::clamp(cmd.left, -1.0, 1.0);
  const double n = std::hypot(f, l);
  if (n > 1.0) {  // diagonal is not faster than straight
    f /= n;
    l /= n;
  }
  const double v = (cmd.run ? cfg.run_speed_mps : cfg.walk_speed_mps) * dt;
  const double c = std::cos(out.yaw);
  const double sn = std::sin(out.yaw);
  const double dx = v * (c * f - sn * l);
  const double dy = v * (sn * f + c * l);
  if (!blocked(s.x + dx, s.y + dy, cfg, obstacles)) {
    out.x = s.x + dx;
    out.y = s.y + dy;
  } else if (!blocked(s.x + dx, s.y, cfg, obstacles)) {
    out.x = s.x + dx;
  } else if (!blocked(s.x, s.y + dy, cfg, obstacles)) {
    out.y = s.y + dy;
  }
  return out;
}

}  // namespace device_view
