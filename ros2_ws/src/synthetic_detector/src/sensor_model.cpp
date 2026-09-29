#include "synthetic_detector/sensor_model.hpp"

#include <cmath>

namespace synthetic_detector
{

Mat3 skew(const Vec3 & v) noexcept
{
  Mat3 m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

Pose camera_pose(const Pose & body_world, const CameraConfig & cam) noexcept
{
  // FLU body: a positive rotation about +y (left) takes +x towards -z, i.e.
  // pitches the optical axis down.
  const Quat mount(Eigen::AngleAxisd(cam.pitch_down_rad, Vec3::UnitY()));
  Pose c;
  c.p = body_world.p + body_world.q * cam.mount_offset_body;
  c.q = (body_world.q * mount).normalized();
  return c;
}

Visibility evaluate_visibility(
  const Pose & body_true, const Vec3 & entity_world, const CameraConfig & cam,
  const Occluders & occluders) noexcept
{
  Visibility v;
  const Pose c = camera_pose(body_true, cam);
  const Vec3 rel = c.q.conjugate() * (entity_world - c.p);
  v.range_m = rel.norm();
  v.in_range = v.range_m >= cam.min_range_m && v.range_m <= cam.max_range_m;
  const double azimuth = std::atan2(rel.y(), rel.x());
  const double elevation = std::atan2(rel.z(), std::hypot(rel.x(), rel.y()));
  v.in_fov = rel.x() > 0.0 && std::abs(azimuth) <= 0.5 * cam.hfov_rad &&
    std::abs(elevation) <= 0.5 * cam.vfov_rad;
  v.occluded = occluded(c.p, entity_world, occluders);
  return v;
}

Mat3 camera_noise_covariance(
  const Vec3 & los_cam, double range_m, const NoiseConfig & noise) noexcept
{
  const double s_range = noise.range_sigma_base_m + noise.range_sigma_per_m * range_m;
  const double s_cross = noise.cross_sigma_base_m + noise.cross_sigma_per_m * range_m;
  // Anisotropic: s_range along the line of sight, s_cross in the plane across it.
  const Mat3 along = los_cam * los_cam.transpose();
  return s_cross * s_cross * (Mat3::Identity() - along) + s_range * s_range * along;
}

}  // namespace synthetic_detector
