#include "device_view/overlay_geometry.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>

namespace device_view
{

CameraPose camera_on_device(
  const Vec3 & device_position, double device_yaw, const Vec3 & mount_xyz) noexcept
{
  CameraPose c;
  c.q = Quat(Eigen::AngleAxisd(device_yaw, Vec3::UnitZ()));
  c.p = device_position + c.q * mount_xyz;
  return c;
}

std::optional<Vec2> project(
  const PinholeCamera & cam, const CameraPose & pose, const Vec3 & world) noexcept
{
  const Vec3 p = pose.q.conjugate() * (world - pose.p);
  if (p.x() < cam.near_m) {
    return std::nullopt;
  }
  return Vec2(cam.cx - cam.fx * p.y() / p.x(), cam.cy - cam.fy * p.z() / p.x());
}

double iou(const Rect & a, const Rect & b) noexcept
{
  const Rect i{std::max(a.x0, b.x0), std::max(a.y0, b.y0), std::min(a.x1, b.x1),
    std::min(a.y1, b.y1)};
  const double inter = i.area();
  const double uni = a.area() + b.area() - inter;
  return uni > 0.0 ? inter / uni : 0.0;
}

std::optional<Rect> clip(const Rect & r, const PinholeCamera & cam) noexcept
{
  const Rect c{std::max(r.x0, 0.0), std::max(r.y0, 0.0),
    std::min(r.x1, static_cast<double>(cam.width)), std::min(r.y1, static_cast<double>(cam.height))};
  if (c.x1 <= c.x0 || c.y1 <= c.y0) {
    return std::nullopt;
  }
  return c;
}

std::array<Vec3, 8> box_corners(const Vec3 & ground_center, double yaw, const Vec3 & size) noexcept
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hl = 0.5 * size.x();
  const double hw = 0.5 * size.y();
  const std::array<Vec2, 4> base{{{hl, hw}, {-hl, hw}, {-hl, -hw}, {hl, -hw}}};
  std::array<Vec3, 8> out{};
  for (std::size_t i = 0; i < 4; ++i) {
    const double x = ground_center.x() + c * base[i].x() - s * base[i].y();
    const double y = ground_center.y() + s * base[i].x() + c * base[i].y();
    out[i] = Vec3(x, y, ground_center.z());
    out[i + 4] = Vec3(x, y, ground_center.z() + size.z());
  }
  return out;
}

std::optional<ProjectedBox> project_box(
  const PinholeCamera & cam, const CameraPose & pose, const std::array<Vec3, 8> & corners) noexcept
{
  ProjectedBox b;
  b.bounds = {1e18, 1e18, -1e18, -1e18};
  for (std::size_t i = 0; i < 8; ++i) {
    const auto px = project(cam, pose, corners[i]);
    if (!px) {
      return std::nullopt;
    }
    b.corners[i] = *px;
    b.bounds.x0 = std::min(b.bounds.x0, px->x());
    b.bounds.y0 = std::min(b.bounds.y0, px->y());
    b.bounds.x1 = std::max(b.bounds.x1, px->x());
    b.bounds.y1 = std::max(b.bounds.y1, px->y());
  }
  return b;
}

Silhouette person_silhouette(
  const Vec3 & ground_point, double height_m, const Vec3 & camera_position) noexcept
{
  // A 1.75 m reference figure, front view: (lateral, up) in metres, clockwise
  // from the left of the neck, with a gap between the legs.
  static constexpr std::array<std::array<double, 2>, kBodyPoints> kBody{{
    {-0.07, 1.46}, {-0.22, 1.41}, {-0.27, 0.98}, {-0.21, 0.96}, {-0.18, 0.90},
    {-0.17, 0.00}, {-0.04, 0.00}, {0.00, 0.80}, {0.04, 0.00}, {0.17, 0.00},
    {0.18, 0.90}, {0.21, 0.96}, {0.27, 0.98}, {0.22, 1.41}, {0.07, 1.46}}};
  constexpr double kHeadCenter = 1.61;
  constexpr double kHeadRadius = 0.13;
  const double scale = height_m / 1.75;

  // Billboard: lateral axis perpendicular to the camera's horizontal line of sight.
  Vec3 los = ground_point - camera_position;
  los.z() = 0.0;
  const Vec3 lateral = los.norm() > 1e-6 ?
    Vec3(Vec3::UnitZ().cross(los).normalized()) : Vec3::UnitY();

  Silhouette s;
  for (std::size_t i = 0; i < kBodyPoints; ++i) {
    s.body[i] = ground_point + scale * (kBody[i][0] * lateral + kBody[i][1] * Vec3::UnitZ());
  }
  for (std::size_t i = 0; i < kHeadPoints; ++i) {
    const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kHeadPoints);
    s.head[i] = ground_point + scale * ((kHeadRadius * std::cos(a)) * lateral +
      (kHeadCenter + kHeadRadius * std::sin(a)) * Vec3::UnitZ());
  }
  return s;
}

std::array<Vec3, kEllipsePoints> ground_ellipse(
  const Vec2 & center_xy, const Mat2 & cov_xy, double n_sigma, double ground_z) noexcept
{
  const Eigen::SelfAdjointEigenSolver<Mat2> es(cov_xy);
  const Vec2 sig = es.eigenvalues().cwiseMax(0.0).cwiseSqrt();
  const Mat2 axes = es.eigenvectors();
  std::array<Vec3, kEllipsePoints> out{};
  for (std::size_t i = 0; i < kEllipsePoints; ++i) {
    const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kEllipsePoints);
    const Vec2 p = center_xy + n_sigma * (sig[0] * std::cos(a) * axes.col(0) +
      sig[1] * std::sin(a) * axes.col(1));
    out[i] = Vec3(p.x(), p.y(), ground_z);
  }
  return out;
}

Presence presence(bool hidden_from_device, int agents_seeing) noexcept
{
  if (!hidden_from_device) {
    return Presence::kVisibleToDevice;
  }
  return agents_seeing > 0 ? Presence::kSeenThroughWall : Presence::kCoasting;
}

double freshness(double age_s, double fade_s, double floor) noexcept
{
  if (fade_s <= 0.0) {
    return 1.0;
  }
  return std::clamp(1.0 - (1.0 - floor) * age_s / fade_s, floor, 1.0);
}

}  // namespace device_view
