// Geometry of the see-through overlay: where a world-frame track lands in the
// device's camera image, and what to draw there.
//
// ROS- and OpenCV-free, so all of it is unit-tested. The node owns the images
// and the drawing.
//
// Frames
//   world   Gazebo ENU (x east, y north, z up), like every track.
//   camera  Gazebo's camera convention: x along the optical axis, y left, z up.
//           (Not the OpenCV/ROS optical frame, which is z forward, x right,
//           y down.) A pinhole camera then maps a point (x, y, z), x > 0, to
//           u = cx - fx * y / x,   v = cy - fy * z / x.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <span>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace device_view
{

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat2 = Eigen::Matrix2d;
using Quat = Eigen::Quaterniond;

struct PinholeCamera
{
  double fx{457.0};
  double fy{457.0};
  double cx{320.0};
  double cy{240.0};
  int width{640};
  int height{480};
  double near_m{0.1};
};

// Camera pose in the world: position, and rotation camera -> world.
struct CameraPose
{
  Vec3 p{Vec3::Zero()};
  Quat q{Quat::Identity()};
};

// Camera on a level device at `device_xy`, heading `device_yaw` (ENU, from
// +x towards +y), mounted at `mount_xyz` in the device frame (x forward).
[[nodiscard]] CameraPose camera_on_device(
  const Vec3 & device_position, double device_yaw, const Vec3 & mount_xyz) noexcept;

// Pixel of a world point, or nothing if it is behind (or too close to) the camera.
[[nodiscard]] std::optional<Vec2> project(
  const PinholeCamera & cam, const CameraPose & pose, const Vec3 & world) noexcept;

// Axis-aligned image rectangle.
struct Rect
{
  double x0{0};
  double y0{0};
  double x1{0};
  double y1{0};
  [[nodiscard]] double area() const noexcept {return std::max(0.0, x1 - x0) * std::max(0.0, y1 - y0);}
  [[nodiscard]] Vec2 center() const noexcept {return {0.5 * (x0 + x1), 0.5 * (y0 + y1)};}
};
[[nodiscard]] double iou(const Rect & a, const Rect & b) noexcept;
// Clip to the image; nothing if no part of it is inside.
[[nodiscard]] std::optional<Rect> clip(const Rect & r, const PinholeCamera & cam) noexcept;

// The 8 corners of an upright box standing on the ground at `ground_center`,
// rotated by `yaw`, with size (length along yaw, width, height). Order: the 4
// bottom corners counter-clockwise, then the 4 top corners above them.
[[nodiscard]] std::array<Vec3, 8> box_corners(
  const Vec3 & ground_center, double yaw, const Vec3 & size) noexcept;

// The projected box: its 8 corners (only if all are in front of the camera)
// and their bounding rectangle.
struct ProjectedBox
{
  std::array<Vec2, 8> corners{};
  Rect bounds;
};
[[nodiscard]] std::optional<ProjectedBox> project_box(
  const PinholeCamera & cam, const CameraPose & pose, const std::array<Vec3, 8> & corners) noexcept;

// A person's silhouette as a flat cut-out standing at `ground_point`, turned
// to face the camera (a billboard), scaled to `height_m`. Returned in world
// coordinates: kBodyPoints body polygon points, then kHeadPoints around the
// head. Reads as a person at any distance, and costs nothing on the link:
// it is drawn from the track, not measured.
inline constexpr std::size_t kBodyPoints = 15;
inline constexpr std::size_t kHeadPoints = 12;
struct Silhouette
{
  std::array<Vec3, kBodyPoints> body{};
  std::array<Vec3, kHeadPoints> head{};
};
[[nodiscard]] Silhouette person_silhouette(
  const Vec3 & ground_point, double height_m, const Vec3 & camera_position) noexcept;

// Points on the n-sigma ellipse of a horizontal position covariance, on the
// ground plane (z = ground_z) around `center_xy`.
inline constexpr std::size_t kEllipsePoints = 32;
[[nodiscard]] std::array<Vec3, kEllipsePoints> ground_ellipse(
  const Vec2 & center_xy, const Mat2 & cov_xy, double n_sigma, double ground_z) noexcept;

// How the overlay should present a track.
enum class Presence : unsigned char
{
  kVisibleToDevice,   // the device has line of sight itself: drawn plainly
  kSeenThroughWall,   // hidden from the device, agents see it now: the see-through case
  kCoasting,          // hidden, and no agent sees it now: prediction only, drawn as uncertain
};

[[nodiscard]] Presence presence(bool hidden_from_device, int agents_seeing) noexcept;

// Opacity for data of a given age: 1 when fresh, fading linearly to `floor`
// at `fade_s`, so stale tracks look stale.
[[nodiscard]] double freshness(double age_s, double fade_s, double floor) noexcept;

}  // namespace device_view
