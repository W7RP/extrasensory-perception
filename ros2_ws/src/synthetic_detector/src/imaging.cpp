#include "synthetic_detector/imaging.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace synthetic_detector
{

namespace
{
constexpr double kDeg = M_PI / 180.0;
constexpr std::array<SensorSpec, 3> kPresets{{
  {"eo_wide_4k", Spectrum::kVisible, 3840, 2160, 60.0 * kDeg, 6.0, 0.5, 0.1 * kDeg, 1500.0, 0.0},
  {"eo_zoom_30x", Spectrum::kVisible, 1920, 1080, 6.0 * kDeg, 6.0, 0.5, 0.05 * kDeg, 3000.0, 0.0},
  {"thermal_640", Spectrum::kThermal, 640, 512, 32.0 * kDeg, 3.0, 0.5, 0.1 * kDeg, 1500.0, 0.3},
}};
}  // namespace

const SensorSpec * sensor_preset(std::string_view name) noexcept
{
  for (const auto & p : kPresets) {
    if (p.name == name) {
      return &p;
    }
  }
  return nullptr;
}

double pixel_footprint_m(const SensorSpec & s, double range_m) noexcept
{
  return range_m * s.hfov_rad / static_cast<double>(s.width_px);
}

double critical_dimension_m(
  double width_m, double depth_m, double height_m, double depression_rad) noexcept
{
  const double d = std::clamp(depression_rad, 0.0, M_PI / 2);
  const double top = width_m * depth_m;
  const double side = width_m * height_m;
  return std::sqrt(top * std::sin(d) + side * std::cos(d));
}

double pixels_on_target(const SensorSpec & s, double range_m, double critical_m) noexcept
{
  const double fp = pixel_footprint_m(s, range_m);
  return fp > 0.0 ? critical_m / fp : 0.0;
}

double detection_probability(double pixels, double n50_px) noexcept
{
  if (pixels <= 0.0 || n50_px <= 0.0) {
    return 0.0;
  }
  const double r = pixels / n50_px;
  const double e = 2.7 + 0.7 * r;
  const double x = std::pow(r, e);
  return x / (1.0 + x);
}

int johnson_level(double pixels) noexcept
{
  if (pixels >= 12.8) {
    return 3;
  }
  if (pixels >= 8.0) {
    return 2;
  }
  return pixels >= 2.0 ? 1 : 0;
}

CameraConfig camera_config(const SensorSpec & s, double pitch_down_rad) noexcept
{
  CameraConfig c;
  c.hfov_rad = s.hfov_rad;
  c.vfov_rad = 2.0 * std::atan(std::tan(0.5 * s.hfov_rad) * s.height_px / s.width_px);
  c.pitch_down_rad = pitch_down_rad;
  c.mount_offset_body = Vec3::Zero();
  c.min_range_m = 1.0;
  c.max_range_m = s.max_range_m;
  return c;
}

double angular_sigma_rad(const SensorSpec & s) noexcept
{
  const double ifov = s.hfov_rad / static_cast<double>(s.width_px);
  return std::hypot(s.pixel_sigma_px * ifov, s.pointing_sigma_rad);
}

Mat3 geolocation_covariance(const Vec3 & camera, const Vec3 & target, double angular_sigma_rad) noexcept
{
  const Vec3 d = target - camera;
  const double r = d.norm();
  const double horiz = std::hypot(d.x(), d.y());
  const double depression = std::atan2(std::abs(d.z()), horiz);
  const double s_cross = r * angular_sigma_rad;
  const double s_along = r * angular_sigma_rad / std::max(std::sin(depression), 0.05);
  // Horizontal axes: along the line of sight's ground track, and across it.
  Vec3 along = horiz > 1e-6 ? Vec3(d.x() / horiz, d.y() / horiz, 0.0) : Vec3::UnitX();
  const Vec3 across(-along.y(), along.x(), 0.0);
  Mat3 cov = s_along * s_along * along * along.transpose() +
    s_cross * s_cross * across * across.transpose();
  cov(2, 2) = 0.05 * 0.05;  // height comes from the map, not the image
  return cov;
}

std::optional<Vec3> ray_to_plane(const Vec3 & origin, const Vec3 & dir, double height) noexcept
{
  if (std::abs(dir.z()) < 1e-9) {
    return std::nullopt;
  }
  const double t = (height - origin.z()) / dir.z();
  if (t <= 0.0) {
    return std::nullopt;
  }
  return Vec3(origin + t * dir);
}

Pose gimbal_camera_pose(
  const Pose & body_world, const Vec3 & mount_offset_body, double yaw_enu_rad,
  double pitch_down_rad) noexcept
{
  Pose c;
  c.p = body_world.p + body_world.q * mount_offset_body;
  c.q = (Quat(Eigen::AngleAxisd(yaw_enu_rad, Vec3::UnitZ())) *
    Quat(Eigen::AngleAxisd(pitch_down_rad, Vec3::UnitY()))).normalized();
  return c;
}

View view(
  const Pose & cam, const SensorSpec & s, const Vec3 & entity, const Vec3 & entity_size,
  double recall, const Occluders & occluders) noexcept
{
  View v;
  const Vec3 d = entity - cam.p;
  const Vec3 rel = cam.q.conjugate() * d;
  v.vis.range_m = rel.norm();
  v.vis.in_range = v.vis.range_m >= 1.0 && v.vis.range_m <= s.max_range_m;
  const double hfov = s.hfov_rad;
  const double vfov = 2.0 * std::atan(std::tan(0.5 * hfov) * s.height_px / s.width_px);
  // Pinhole: inside the image rectangle, not an angular cone.
  v.vis.in_fov = rel.x() > 0.0 && std::abs(rel.y() / rel.x()) <= std::tan(0.5 * hfov) &&
    std::abs(rel.z() / rel.x()) <= std::tan(0.5 * vfov);
  if (!v.vis.in_fov || !v.vis.in_range) {
    return v;
  }
  const Blockage b = blockage(cam.p, entity, occluders);
  v.canopies = b.canopies;
  const double through = b.canopies == 0 ? 1.0 :
    std::pow(s.canopy_transmission, static_cast<double>(b.canopies));
  v.vis.occluded = b.solid || through <= 0.0;
  v.depression_rad = std::atan2(-d.z(), std::hypot(d.x(), d.y()));
  v.pixels = pixels_on_target(s, v.vis.range_m, critical_dimension_m(
      entity_size.x(), entity_size.y(), entity_size.z(), v.depression_rad));
  if (v.vis.visible()) {
    v.p_detect = std::clamp(recall, 0.0, 1.0) * detection_probability(v.pixels, s.n50_px) * through;
  }
  return v;
}

}  // namespace synthetic_detector
