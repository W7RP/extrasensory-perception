// Synthetic detector model: what one agent's camera + detector would report
// about an entity, computed from ground truth.
//
// ROS-free on purpose (the node owns I/O and randomness seeding), so every
// rule below is unit-tested without a simulator.
//
// Frames
//   world   Gazebo world, ENU (x east, y north, z up). Every output is in it.
//   body    the agent's body, FLU (x forward, y left, z up), as Gazebo models
//           and REP-103 odometry use.
//   camera  body rotated by the mount pitch (positive = nose down) and offset by
//           the mount position; x is the optical axis, y left, z up.
//
// The model, in order:
//   1. Visibility (truth): the entity's reference point is inside the camera's
//      horizontal/vertical field of view, within [min_range, max_range], and the
//      straight line from the camera to it crosses no occluder.
//   2. Dropout: a visible entity is still missed with probability p_miss per
//      frame (the detector's recall is not 1).
//   3. Measurement: the TRUE camera-frame vector to the entity, plus Gaussian
//      noise that grows with range (range error along the line of sight,
//      angular error across it), is rotated into the world with the agent's
//      ESTIMATED attitude and added to its ESTIMATED camera position. So the
//      agent's own navigation error lands in the detection, exactly as it would
//      for a real camera detector projecting an image box into the world.
//   4. Covariance: the noise covariance, rotated the same way, plus the
//      estimator's reported position covariance, plus its attitude uncertainty
//      mapped through the lever arm: a small attitude error dtheta moves the
//      detection by dtheta x r, so its covariance adds [r]x Sigma_att [r]x^T.
//      That term matters: 0.5 deg of yaw error is 17 cm at 20 m.
#pragma once

#include <algorithm>
#include <cstdint>
#include <random>
#include <span>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "synthetic_detector/geometry.hpp"

namespace synthetic_detector
{

struct CameraConfig
{
  double hfov_rad{1.5708};          // full horizontal field of view
  double vfov_rad{1.0472};          // full vertical field of view
  double pitch_down_rad{0.2618};    // mount pitch below the body x axis
  Vec3 mount_offset_body{0.12, 0.0, -0.05};  // camera position in the body frame [m]
  double min_range_m{0.5};
  double max_range_m{25.0};
};

struct NoiseConfig
{
  // 1-sigma, in the camera frame: sigma = base + per_m * range.
  double range_sigma_base_m{0.15};
  double range_sigma_per_m{0.02};
  double cross_sigma_base_m{0.05};
  double cross_sigma_per_m{0.005};  // i.e. 5 mrad of angular error
  double p_miss{0.1};               // per-frame dropout of a visible entity
};

struct Visibility
{
  bool in_fov{false};
  bool in_range{false};
  bool occluded{false};
  double range_m{0.0};
  [[nodiscard]] bool visible() const noexcept {return in_fov && in_range && !occluded;}
};

struct Measurement
{
  Vec3 position_world{Vec3::Zero()};
  Mat3 covariance_world{Mat3::Zero()};
  double confidence{0.0};
};

// World pose of the camera, given the body's world pose.
[[nodiscard]] Pose camera_pose(const Pose & body_world, const CameraConfig & cam) noexcept;

// Step 1. `entity_world` is the entity's reference point (its centre).
[[nodiscard]] Visibility evaluate_visibility(
  const Pose & body_true, const Vec3 & entity_world, const CameraConfig & cam,
  const Occluders & occluders) noexcept;

// Camera-frame noise covariance for an entity at `range_m` along the optical
// direction `los_cam` (unit vector, camera frame).
[[nodiscard]] Mat3 camera_noise_covariance(
  const Vec3 & los_cam, double range_m, const NoiseConfig & noise) noexcept;

// The agent's own navigation uncertainty, as its estimator reports it.
struct NavUncertainty
{
  Mat3 position_cov{Mat3::Zero()};       // world frame [m^2]
  Vec3 attitude_var{Vec3::Zero()};       // roll, pitch, yaw error variances [rad^2]
};

[[nodiscard]] Mat3 skew(const Vec3 & v) noexcept;

// Steps 3 and 4 (the caller has already decided the entity is visible and not
// dropped).
template<typename Rng>
[[nodiscard]] Measurement synthesize(
  const Pose & body_true, const Pose & body_est, const NavUncertainty & nav,
  const Vec3 & entity_world, const CameraConfig & cam, const NoiseConfig & noise, Rng & rng)
{
  const Pose cam_true = camera_pose(body_true, cam);
  const Pose cam_est = camera_pose(body_est, cam);
  const Vec3 rel_cam = cam_true.q.conjugate() * (entity_world - cam_true.p);
  const double range = rel_cam.norm();
  const Vec3 los = range > 1e-9 ? Vec3(rel_cam / range) : Vec3::UnitX();
  const Mat3 cov_cam = camera_noise_covariance(los, range, noise);

  // Sample from N(0, cov_cam) with its Cholesky factor (3x3, fixed size).
  const Eigen::LLT<Mat3> llt(cov_cam);
  std::normal_distribution<double> n01(0.0, 1.0);
  const Vec3 z(n01(rng), n01(rng), n01(rng));
  const Vec3 noisy_cam = rel_cam + llt.matrixL() * z;

  const Mat3 R = cam_est.q.toRotationMatrix();
  Measurement m;
  m.position_world = cam_est.p + R * noisy_cam;
  // Attitude error variances are body-frame; for the small tilts of level
  // flight, treating them as world-frame roll/pitch/yaw is accurate enough.
  const Mat3 S = skew(R * rel_cam);
  m.covariance_world = R * cov_cam * R.transpose() + nav.position_cov +
    S * nav.attitude_var.asDiagonal() * S.transpose();
  m.confidence = std::clamp(1.0 - 0.5 * range / cam.max_range_m, 0.3, 1.0);
  return m;
}

}  // namespace synthetic_detector
