// Small geometry kit for the synthetic detector: poses and the shapes that
// block a line of sight (oriented boxes, capped cylinders, spheres), and the
// one query occlusion needs: does a line segment pass through any of them?
// Fixed-size Eigen; the queries never allocate (an Occluders set is built
// once, from the world file).
#pragma once

#include <optional>
#include <span>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace synthetic_detector
{

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

struct Pose
{
  Vec3 p{Vec3::Zero()};
  Quat q{Quat::Identity()};  // rotates this frame's vectors into the parent frame
};

// Oriented bounding box: centre and orientation in the world, half sizes along
// its own axes.
struct Obb
{
  Vec3 center{Vec3::Zero()};
  Quat q{Quat::Identity()};
  Vec3 half_extents{Vec3::Zero()};
};

// Capped cylinder: axis along its own z (a tree trunk, a post, a tank), centre
// halfway up.
struct Cylinder
{
  Vec3 center{Vec3::Zero()};
  Quat q{Quat::Identity()};
  double radius{0.0};
  double half_length{0.0};
};

// Sphere (a tree canopy): solid to visible light, partly transparent to a
// thermal camera (see `blockage` and imaging.hpp).
struct Sphere
{
  Vec3 center{Vec3::Zero()};
  double radius{0.0};
};

// Everything that can block a line of sight in a world.
struct Occluders
{
  std::vector<Obb> boxes;
  std::vector<Cylinder> cylinders;
  std::vector<Sphere> spheres;
  [[nodiscard]] std::size_t size() const noexcept
  {
    return boxes.size() + cylinders.size() + spheres.size();
  }
};

// True if the segment a -> b intersects the shape (touching counts).
// Box: slab test in the box's own frame. Cylinder: the segment's interval
// inside the infinite cylinder, clipped to the caps. Sphere: closest point.
[[nodiscard]] bool segment_intersects(const Vec3 & a, const Vec3 & b, const Obb & box) noexcept;
[[nodiscard]] bool segment_intersects(const Vec3 & a, const Vec3 & b, const Cylinder & c) noexcept;
[[nodiscard]] bool segment_intersects(const Vec3 & a, const Vec3 & b, const Sphere & s) noexcept;

// Index of the first box the segment intersects, if any.
[[nodiscard]] std::optional<std::size_t> first_occluder(
  const Vec3 & a, const Vec3 & b, std::span<const Obb> boxes) noexcept;

// True if anything in the set blocks the segment.
[[nodiscard]] bool occluded(const Vec3 & a, const Vec3 & b, const Occluders & occ) noexcept;

// What lies on the segment: whether a solid shape (box, cylinder) blocks it,
// and how many canopies (spheres) it passes through.
struct Blockage
{
  bool solid{false};
  int canopies{0};
};
[[nodiscard]] Blockage blockage(const Vec3 & a, const Vec3 & b, const Occluders & occ) noexcept;

}  // namespace synthetic_detector
