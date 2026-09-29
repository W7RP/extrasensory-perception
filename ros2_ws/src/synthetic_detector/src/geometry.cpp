#include "synthetic_detector/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace synthetic_detector
{

bool segment_intersects(const Vec3 & a, const Vec3 & b, const Obb & box) noexcept
{
  // Into the box frame, where it is axis-aligned and centred on the origin.
  const Quat qi = box.q.conjugate();
  const Vec3 p0 = qi * (a - box.center);
  const Vec3 d = qi * (b - a);
  double t_min = 0.0;
  double t_max = 1.0;
  for (int i = 0; i < 3; ++i) {
    const double h = box.half_extents[i];
    if (std::abs(d[i]) < 1e-12) {
      if (p0[i] < -h || p0[i] > h) {
        return false;  // parallel to this slab and outside it
      }
      continue;
    }
    double t0 = (-h - p0[i]) / d[i];
    double t1 = (h - p0[i]) / d[i];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    t_min = std::max(t_min, t0);
    t_max = std::min(t_max, t1);
    if (t_min > t_max) {
      return false;
    }
  }
  return true;
}

std::optional<std::size_t> first_occluder(
  const Vec3 & a, const Vec3 & b, std::span<const Obb> boxes) noexcept
{
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    if (segment_intersects(a, b, boxes[i])) {
      return i;
    }
  }
  return std::nullopt;
}

}  // namespace synthetic_detector
