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

bool segment_intersects(const Vec3 & a, const Vec3 & b, const Cylinder & c) noexcept
{
  const Quat qi = c.q.conjugate();
  const Vec3 p = qi * (a - c.center);
  const Vec3 d = qi * (b - a);
  // Interval of t in [0, 1] with the point inside the caps' slab.
  double t_min = 0.0;
  double t_max = 1.0;
  if (std::abs(d.z()) < 1e-12) {
    if (std::abs(p.z()) > c.half_length) {
      return false;
    }
  } else {
    double t0 = (-c.half_length - p.z()) / d.z();
    double t1 = (c.half_length - p.z()) / d.z();
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    t_min = std::max(t_min, t0);
    t_max = std::min(t_max, t1);
    if (t_min > t_max) {
      return false;
    }
  }
  // Inside the infinite cylinder: (px + t dx)^2 + (py + t dy)^2 <= r^2.
  const double A = d.x() * d.x() + d.y() * d.y();
  const double B = 2.0 * (p.x() * d.x() + p.y() * d.y());
  const double C = p.x() * p.x() + p.y() * p.y() - c.radius * c.radius;
  if (A < 1e-12) {
    return C <= 0.0;  // parallel to the axis: inside the circle or not
  }
  const double disc = B * B - 4.0 * A * C;
  if (disc < 0.0) {
    return false;
  }
  const double sq = std::sqrt(disc);
  const double r0 = (-B - sq) / (2.0 * A);
  const double r1 = (-B + sq) / (2.0 * A);
  return std::max(t_min, r0) <= std::min(t_max, r1);
}

bool segment_intersects(const Vec3 & a, const Vec3 & b, const Sphere & s) noexcept
{
  const Vec3 d = b - a;
  const double len2 = d.squaredNorm();
  const double t = len2 > 1e-24 ? std::clamp((s.center - a).dot(d) / len2, 0.0, 1.0) : 0.0;
  return (a + t * d - s.center).squaredNorm() <= s.radius * s.radius;
}

bool occluded(const Vec3 & a, const Vec3 & b, const Occluders & occ) noexcept
{
  for (const auto & x : occ.boxes) {
    if (segment_intersects(a, b, x)) {
      return true;
    }
  }
  for (const auto & x : occ.cylinders) {
    if (segment_intersects(a, b, x)) {
      return true;
    }
  }
  for (const auto & x : occ.spheres) {
    if (segment_intersects(a, b, x)) {
      return true;
    }
  }
  return false;
}

}  // namespace synthetic_detector
