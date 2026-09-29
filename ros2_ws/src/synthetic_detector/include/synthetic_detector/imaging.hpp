// Imaging model for overhead sensors: what a camera of a given spec can
// resolve, detect and geolocate. Used by the high overwatch unit (Phase 4);
// the Phase 1-3 low agents keep the simpler range model in sensor_model.hpp.
//
// ROS-free and unit-tested. The chain, per camera and entity:
//
//   1. Pixels on target. The entity's critical dimension is the square root
//      of its projected area seen from the camera (the usual convention for
//      target acquisition): a standing person, 0.5 x 0.3 x 1.75 m, is 0.39 m
//      seen straight down and ~0.85 m at 45 deg. Pixels on target = that
//      divided by the ground footprint of one pixel at the slant range.
//      Looking straight down is therefore honestly harder than looking at an
//      angle.
//   2. Detection probability from pixels on target: the target transfer
//      probability function (TTPF) of the Johnson-criteria models,
//        P = (N/N50)^E / (1 + (N/N50)^E),   E = 2.7 + 0.7 N/N50,
//      with N50 the pixels across the critical dimension at which a detector
//      finds the target half the time. It is a property of the sensor and the
//      detector: an automated detector on visible imagery needs more pixels
//      than a hot blob on a thermal image needs.
//   3. Johnson levels for reporting: detect / recognise / identify at 2 / 8 /
//      12.8 pixels across the critical dimension (Johnson's 1.0 / 4.0 / 6.4
//      cycles, two pixels per cycle).
//   4. Geolocation the way airborne systems do it: the ray from the camera
//      through the target's pixel, intersected with the horizontal plane at the
//      target's reference height (the map's ground, known). An angular error
//      dtheta (pixel localisation plus gimbal/INS pointing) moves the result
//      by R dtheta across the line of sight and R dtheta / sin(depression)
//      along it, so shallow looks stretch the error.
//   5. What one camera makes of one entity (`view`): geometric visibility
//      (field of view, range, solid occluders), canopies on the line of sight
//      (each passed with the sensor's canopy transmission), pixels on target
//      and the resulting detection probability,
//        p_detect = recall * TTPF(pixels) * transmission^canopies.
#pragma once

#include <optional>
#include <random>
#include <string_view>

#include "synthetic_detector/geometry.hpp"
#include "synthetic_detector/sensor_model.hpp"

namespace synthetic_detector
{

enum class Spectrum : unsigned char
{
  kVisible,
  kThermal,
};

struct SensorSpec
{
  std::string_view name;
  Spectrum spectrum{Spectrum::kVisible};
  int width_px{3840};
  int height_px{2160};
  double hfov_rad{1.047};
  double n50_px{6.0};             // pixels across the critical dimension for P = 0.5
  double pixel_sigma_px{0.5};     // how precisely the detector finds the target in the image
  double pointing_sigma_rad{0.0017};  // gimbal + inertial attitude error (0.1 deg)
  double max_range_m{1500.0};     // haze / atmosphere: nothing beyond
  double canopy_transmission{0.0};  // chance a canopy does NOT hide the target
};

// Presets typical of commercial payloads (no specific product):
//   eo_wide_4k    visible, 3840 x 2160, 60 deg (a wide-angle mapping/search camera)
//   eo_zoom_30x   visible, 1920 x 1080, zoomed to 6 deg (a 30x zoom block near the
//                 long end of its range; the render uses the same fixed step)
//   thermal_640   long-wave infrared, 640 x 512, 32 deg; people are hot against
//                 cool ground (N50 3 px), and sparse canopy lets some through
[[nodiscard]] const SensorSpec * sensor_preset(std::string_view name) noexcept;

// Ground footprint of one pixel at slant range r [m].
[[nodiscard]] double pixel_footprint_m(const SensorSpec & s, double range_m) noexcept;

// Critical dimension [m] of an upright box (width, depth, height) seen at a
// depression angle (0 = level, pi/2 = straight down).
[[nodiscard]] double critical_dimension_m(
  double width_m, double depth_m, double height_m, double depression_rad) noexcept;

[[nodiscard]] double pixels_on_target(
  const SensorSpec & s, double range_m, double critical_m) noexcept;

[[nodiscard]] double detection_probability(double pixels, double n50_px) noexcept;

// 0 none, 1 detect, 2 recognise, 3 identify.
[[nodiscard]] int johnson_level(double pixels) noexcept;

// The camera configuration (field of view, range) a spec implies, for the
// visibility test.
[[nodiscard]] CameraConfig camera_config(const SensorSpec & s, double pitch_down_rad) noexcept;

// Total 1-sigma angular error of one detection [rad].
[[nodiscard]] double angular_sigma_rad(const SensorSpec & s) noexcept;

// World-frame covariance of a ground-intersection geolocation from `camera`
// to `target` with angular error sigma.
[[nodiscard]] Mat3 geolocation_covariance(
  const Vec3 & camera, const Vec3 & target, double angular_sigma_rad) noexcept;

// Intersect the ray origin + t * dir (t > 0) with the plane z = height.
[[nodiscard]] std::optional<Vec3> ray_to_plane(
  const Vec3 & origin, const Vec3 & dir, double height) noexcept;

// World pose of a gimbal camera: stabilised in the world, so only its
// position follows the airframe. x is the optical axis (Gazebo convention).
[[nodiscard]] Pose gimbal_camera_pose(
  const Pose & body_world, const Vec3 & mount_offset_body, double yaw_enu_rad,
  double pitch_down_rad) noexcept;

// Step 5, for one camera at world pose `cam` and one entity whose reference
// point (its centre) is `entity`, of size (width, depth, height).
struct View
{
  Visibility vis;             // occluded = a solid shape, or a canopy the sensor cannot see through
  int canopies{0};
  double depression_rad{0.0};
  double pixels{0.0};
  double p_detect{0.0};       // 0 unless vis.visible()
};
[[nodiscard]] View view(
  const Pose & cam, const SensorSpec & s, const Vec3 & entity, const Vec3 & entity_size,
  double recall, const Occluders & occluders) noexcept;

// A detection by ground intersection. The TRUE direction to the target,
// perturbed by `sigma` in two directions across it, is taken as seen from the
// ESTIMATED camera pose (the camera's own navigation), and intersected with
// the target's reference-height plane. So navigation bias and attitude error
// land in the result the way they would for a real payload.
template<typename Rng>
[[nodiscard]] std::optional<Measurement> geolocate(
  const Pose & cam_true, const Pose & cam_est, const Vec3 & target, double sigma,
  const NavUncertainty & nav, Rng & rng)
{
  const Vec3 d_world = target - cam_true.p;
  if (d_world.norm() < 1e-6) {
    return std::nullopt;
  }
  Vec3 d_cam = cam_true.q.conjugate() * d_world.normalized();
  // Two unit vectors across the line of sight, in the camera frame.
  const Vec3 a = d_cam.unitOrthogonal();
  const Vec3 b = d_cam.cross(a);
  std::normal_distribution<double> n01(0.0, 1.0);
  d_cam = (d_cam + sigma * n01(rng) * a + sigma * n01(rng) * b).normalized();
  const auto hit = ray_to_plane(cam_est.p, cam_est.q * d_cam, target.z());
  if (!hit) {
    return std::nullopt;
  }
  Measurement m;
  m.position_world = *hit;
  const Mat3 S = skew(*hit - cam_est.p);
  m.covariance_world = geolocation_covariance(cam_est.p, *hit, sigma) + nav.position_cov +
    S * nav.attitude_var.asDiagonal() * S.transpose();
  m.confidence = 1.0;
  return m;
}

}  // namespace synthetic_detector
