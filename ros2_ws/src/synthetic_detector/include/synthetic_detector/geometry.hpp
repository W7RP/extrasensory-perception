// Small geometry kit for the synthetic detector: poses and oriented boxes, and
// the one query occlusion needs: does a line segment pass through a box?
// Fixed-size Eigen throughout; nothing here allocates.
#pragma once

#include <optional>
#include <span>

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

// True if the segment a -> b intersects the box (touching counts). Slab test in
// the box's own frame.
[[nodiscard]] bool segment_intersects(const Vec3 & a, const Vec3 & b, const Obb & box) noexcept;

// Index of the first box the segment intersects, if any.
[[nodiscard]] std::optional<std::size_t> first_occluder(
  const Vec3 & a, const Vec3 & b, std::span<const Obb> boxes) noexcept;

}  // namespace synthetic_detector
