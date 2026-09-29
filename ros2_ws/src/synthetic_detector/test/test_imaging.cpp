#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "synthetic_detector/imaging.hpp"

using namespace synthetic_detector;

namespace
{
constexpr double kDeg = M_PI / 180.0;

Pose looking_down_at(const Vec3 & cam, const Vec3 & target)
{
  // Camera x axis along the line of sight (Gazebo convention), level y.
  const Vec3 x = (target - cam).normalized();
  Vec3 y = Vec3::UnitZ().cross(x);
  if (y.norm() < 1e-6) {
    y = Vec3::UnitY();
  }
  y.normalize();
  const Vec3 z = x.cross(y);
  Mat3 R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  Pose p;
  p.p = cam;
  p.q = Quat(R);
  return p;
}
}  // namespace

TEST(Imaging, PresetsExist)
{
  ASSERT_NE(sensor_preset("eo_wide_4k"), nullptr);
  ASSERT_NE(sensor_preset("eo_zoom_30x"), nullptr);
  ASSERT_NE(sensor_preset("thermal_640"), nullptr);
  EXPECT_EQ(sensor_preset("nope"), nullptr);
  EXPECT_EQ(sensor_preset("thermal_640")->spectrum, Spectrum::kThermal);
}

TEST(Imaging, PixelsOnAPersonFromOverhead)
{
  const SensorSpec & wide = *sensor_preset("eo_wide_4k");
  // A standing person seen straight down: 0.5 x 0.3 m of shoulders and head.
  const double nadir = critical_dimension_m(0.5, 0.3, 1.75, M_PI / 2);
  EXPECT_NEAR(nadir, std::sqrt(0.15), 1e-12);
  // ... and at 45 deg the body shows, so the critical dimension more than doubles.
  EXPECT_GT(critical_dimension_m(0.5, 0.3, 1.75, M_PI / 4), 2.0 * nadir);
  // 4K over 60 deg at 100 m: 2.7 cm per pixel, 14 pixels across the person.
  EXPECT_NEAR(pixel_footprint_m(wide, 100.0), 100.0 * 60 * kDeg / 3840, 1e-12);
  EXPECT_NEAR(pixels_on_target(wide, 100.0, nadir), 14.2, 0.1);
  // The zoom camera at 300 m still identifies.
  const SensorSpec & zoom = *sensor_preset("eo_zoom_30x");
  EXPECT_EQ(johnson_level(pixels_on_target(zoom, 300.0, nadir)), 3);
}

TEST(Imaging, TargetTransferProbability)
{
  EXPECT_DOUBLE_EQ(detection_probability(6.0, 6.0), 0.5);
  EXPECT_LT(detection_probability(3.0, 6.0), 0.2);
  EXPECT_NEAR(detection_probability(12.0, 6.0), 0.945, 0.001);  // 2^4.1 / (1 + 2^4.1)
  EXPECT_DOUBLE_EQ(detection_probability(0.0, 6.0), 0.0);
  double last = 0.0;
  for (double px = 0.5; px < 30.0; px += 0.5) {
    const double p = detection_probability(px, 6.0);
    EXPECT_GE(p, last);
    last = p;
  }
  EXPECT_EQ(johnson_level(1.0), 0);
  EXPECT_EQ(johnson_level(3.0), 1);
  EXPECT_EQ(johnson_level(9.0), 2);
  EXPECT_EQ(johnson_level(20.0), 3);
}

TEST(Imaging, CameraConfigFollowsTheSpec)
{
  const CameraConfig c = camera_config(*sensor_preset("eo_wide_4k"), 1.0);
  EXPECT_NEAR(c.hfov_rad, 60 * kDeg, 1e-12);
  EXPECT_NEAR(c.vfov_rad, 2 * std::atan(std::tan(30 * kDeg) * 2160.0 / 3840.0), 1e-12);
  EXPECT_DOUBLE_EQ(c.pitch_down_rad, 1.0);
}

TEST(Imaging, RayToPlane)
{
  const auto hit = ray_to_plane({0, 0, 100}, Vec3(1, 0, -1).normalized(), 0.9);
  ASSERT_TRUE(hit);
  EXPECT_NEAR(hit->x(), 99.1, 1e-9);
  EXPECT_NEAR(hit->z(), 0.9, 1e-12);
  EXPECT_FALSE(ray_to_plane({0, 0, 100}, Vec3(1, 0, 1).normalized(), 0.9));  // looking up
}

TEST(Imaging, GeolocationIsExactWithoutErrorsAndCarriesNavigationBias)
{
  const Vec3 target(30, 20, 0.9);
  const Pose truth = looking_down_at({0, 0, 100}, target);
  std::mt19937_64 rng(1);
  auto m = geolocate(truth, truth, target, 0.0, {}, rng);
  ASSERT_TRUE(m);
  EXPECT_NEAR((m->position_world - target).norm(), 0.0, 1e-9);
  // A 1.5 m GPS bias of the camera moves the detection by the same 1.5 m.
  Pose biased = truth;
  biased.p += Vec3(1.5, 0, 0);
  m = geolocate(truth, biased, target, 0.0, {}, rng);
  ASSERT_TRUE(m);
  EXPECT_NEAR(m->position_world.x() - target.x(), 1.5, 1e-9);
  // 0.1 deg of attitude error at ~105 m slant range: ~0.2 m on the ground.
  Pose tilted = truth;
  tilted.q = (truth.q * Quat(Eigen::AngleAxisd(0.1 * kDeg, Vec3::UnitZ()))).normalized();
  m = geolocate(truth, tilted, target, 0.0, {}, rng);
  ASSERT_TRUE(m);
  EXPECT_NEAR((m->position_world - target).norm(), 0.1 * kDeg * (target - truth.p).norm(), 0.05);
}

TEST(Imaging, GeolocationCovarianceMatchesSamplesAndStretchesAtShallowLooks)
{
  const Vec3 target(60, 0, 0.9);
  const Pose cam = looking_down_at({0, 0, 60}, target);  // 45 deg depression
  const double sigma = 2e-3;
  std::mt19937_64 rng(3);
  std::vector<Vec3> pts;
  Vec3 mean = Vec3::Zero();
  for (int i = 0; i < 20000; ++i) {
    const auto m = geolocate(cam, cam, target, sigma, {}, rng);
    ASSERT_TRUE(m);
    pts.push_back(m->position_world);
    mean += m->position_world;
  }
  mean /= static_cast<double>(pts.size());
  double vx = 0, vy = 0;
  for (const auto & p : pts) {
    vx += (p.x() - mean.x()) * (p.x() - mean.x());
    vy += (p.y() - mean.y()) * (p.y() - mean.y());
  }
  vx /= static_cast<double>(pts.size());
  vy /= static_cast<double>(pts.size());
  const Mat3 cov = geolocation_covariance(cam.p, target, sigma);
  EXPECT_NEAR(vx, cov(0, 0), 0.1 * cov(0, 0));  // along the line of sight
  EXPECT_NEAR(vy, cov(1, 1), 0.1 * cov(1, 1));  // across it
  EXPECT_GT(cov(0, 0), cov(1, 1));              // stretched along it
  // Straight down, both axes are the same.
  const Mat3 nadir = geolocation_covariance({0, 0, 100}, {0.001, 0, 0.9}, sigma);
  EXPECT_NEAR(nadir(0, 0), nadir(1, 1), 1e-6);
}

TEST(Imaging, GimbalCameraPoseIgnoresAirframeAttitude)
{
  Pose body;
  body.p = {10, 0, 100};
  body.q = Quat(Eigen::AngleAxisd(0.3, Vec3::UnitX()));  // rolled 17 deg
  const Pose c = gimbal_camera_pose(body, Vec3::Zero(), M_PI / 2, M_PI / 4);
  const Vec3 axis = c.q * Vec3::UnitX();
  EXPECT_NEAR(axis.x(), 0.0, 1e-12);
  EXPECT_NEAR(axis.y(), M_SQRT1_2, 1e-12);  // north
  EXPECT_NEAR(axis.z(), -M_SQRT1_2, 1e-12);  // 45 deg down
}

TEST(Imaging, ViewFromHighUp)
{
  const SensorSpec & wide = *sensor_preset("eo_wide_4k");
  const SensorSpec & zoom = *sensor_preset("eo_zoom_30x");
  const SensorSpec & ir = *sensor_preset("thermal_640");
  const Vec3 size(0.5, 0.3, 1.75);
  const Vec3 entity(60, 0, 0.9);
  const Pose cam = looking_down_at({0, 0, 100}, entity);
  Occluders none;
  const View w = view(cam, wide, entity, size, 0.9, none);
  ASSERT_TRUE(w.vis.visible());
  EXPECT_NEAR(w.depression_rad, std::atan2(99.1, 60.0), 1e-9);
  // 4K over 60 deg at 116 m: 3 cm pixels, ~24 across a person seen at 59 deg.
  EXPECT_NEAR(w.pixels, 24.0, 1.0);
  EXPECT_NEAR(w.p_detect, 0.9, 1e-3);
  const View z = view(cam, zoom, entity, size, 0.9, none);
  EXPECT_NEAR(z.pixels / w.pixels, (60.0 / 3840) / (6.0 / 1920), 1e-9);
  EXPECT_NEAR(z.p_detect, 0.9, 1e-6);
  // Out of the narrow field: 10 deg off axis.
  const Pose off = looking_down_at({0, 0, 100}, {60, 20, 0.9});
  EXPECT_FALSE(view(off, zoom, entity, size, 0.9, none).vis.in_fov);
  EXPECT_TRUE(view(off, wide, entity, size, 0.9, none).vis.in_fov);

  // Under a canopy: hidden from the visible cameras, sometimes seen in thermal.
  Occluders tree;
  tree.spheres.push_back({entity + Vec3(0, 0, 3), 2.0});
  EXPECT_TRUE(view(cam, wide, entity, size, 0.9, tree).vis.occluded);
  const View t = view(cam, ir, entity, size, 0.9, tree);
  EXPECT_FALSE(t.vis.occluded);
  EXPECT_EQ(t.canopies, 1);
  EXPECT_NEAR(t.p_detect, 0.9 * detection_probability(t.pixels, ir.n50_px) * 0.3, 1e-12);
  // A wall hides it from everything.
  Occluders wall;
  wall.boxes.push_back({entity + Vec3(0, 0, 3), Quat::Identity(), {2, 2, 0.1}});
  EXPECT_TRUE(view(cam, ir, entity, size, 0.9, wall).vis.occluded);
  EXPECT_DOUBLE_EQ(view(cam, ir, entity, size, 0.9, wall).p_detect, 0.0);
}
