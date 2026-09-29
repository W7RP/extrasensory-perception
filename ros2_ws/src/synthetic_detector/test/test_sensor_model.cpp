#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "synthetic_detector/geometry.hpp"
#include "synthetic_detector/sdf_occluders.hpp"
#include "synthetic_detector/sensor_model.hpp"

using namespace synthetic_detector;

namespace
{
Obb wall()
{
  // The Phase 1 building: 2 x 20 x 6 m, centred at (0, 0, 3).
  Obb b;
  b.center = Vec3(0, 0, 3);
  b.half_extents = Vec3(1, 10, 3);
  return b;
}

Pose facing(double x, double y, double z, double yaw)
{
  Pose p;
  p.p = Vec3(x, y, z);
  p.q = Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ()));
  return p;
}

CameraConfig ideal_camera()
{
  CameraConfig c;
  c.mount_offset_body = Vec3::Zero();
  c.pitch_down_rad = 0.0;
  return c;
}
}  // namespace

TEST(Geometry, SegmentThroughMissAndTouch)
{
  const Obb b = wall();
  EXPECT_TRUE(segment_intersects({-5, 0, 2}, {5, 0, 2}, b));    // straight through
  EXPECT_FALSE(segment_intersects({-5, 12, 2}, {5, 12, 2}, b)); // past its end
  EXPECT_FALSE(segment_intersects({-5, 0, 7}, {5, 0, 7}, b));   // over the roof
  EXPECT_FALSE(segment_intersects({-5, 0, 2}, {-2, 0, 2}, b));  // stops short
  EXPECT_TRUE(segment_intersects({-5, 0, 2}, {-1, 0, 2}, b));   // touches the face
  EXPECT_TRUE(segment_intersects({0, 0, 2}, {0.5, 0, 2}, b));   // starts inside
  // Parallel to a face, outside and inside the slab.
  EXPECT_FALSE(segment_intersects({2, -20, 2}, {2, 20, 2}, b));
  EXPECT_TRUE(segment_intersects({0.5, -20, 2}, {0.5, 20, 2}, b));
}

TEST(Geometry, RotatedBox)
{
  Obb b;
  b.half_extents = Vec3(2, 0.1, 1);  // a thin slab along x ...
  b.q = Quat(Eigen::AngleAxisd(M_PI / 2, Vec3::UnitZ()));  // ... turned to lie along y
  EXPECT_TRUE(segment_intersects({-1, 0, 0}, {1, 0, 0}, b));
  EXPECT_FALSE(segment_intersects({-1, 3, 0}, {1, 3, 0}, b));
  EXPECT_TRUE(segment_intersects({-1, 1.9, 0}, {1, 1.9, 0}, b));
  const std::vector<Obb> boxes{wall(), b};
  EXPECT_EQ(first_occluder({-5, 0, 2}, {5, 0, 2}, boxes), 0U);
  EXPECT_FALSE(first_occluder({-5, 30, 2}, {5, 30, 2}, boxes).has_value());
}

TEST(Camera, MountPitchPointsTheAxisDown)
{
  CameraConfig c = ideal_camera();
  c.pitch_down_rad = M_PI / 6;
  c.mount_offset_body = Vec3(0.1, 0, 0);
  const Pose cam = camera_pose(facing(1, 2, 3, M_PI / 2), c);  // facing north
  const Vec3 axis = cam.q * Vec3::UnitX();
  EXPECT_NEAR(axis.x(), 0.0, 1e-12);
  EXPECT_NEAR(axis.y(), std::cos(M_PI / 6), 1e-12);
  EXPECT_NEAR(axis.z(), -std::sin(M_PI / 6), 1e-12);
  EXPECT_NEAR(cam.p.x(), 1.0, 1e-12);
  EXPECT_NEAR(cam.p.y(), 2.1, 1e-12);
}

TEST(Visibility, FovRangeAndOcclusion)
{
  const std::vector<Obb> occ{wall()};
  CameraConfig c = ideal_camera();
  c.hfov_rad = M_PI / 2;
  c.vfov_rad = M_PI / 3;
  c.max_range_m = 25;
  const Pose south = facing(0, -15, 4, M_PI / 2);  // south of the wall's end, facing north

  // Past the wall's end, ahead and slightly below: visible.
  auto v = evaluate_visibility(facing(6, -15, 4, M_PI / 2), {4, -6, 0.9}, c, occ);
  EXPECT_TRUE(v.visible());
  EXPECT_NEAR(v.range_m, std::sqrt(4 + 81 + 3.1 * 3.1), 1e-9);

  // Line of sight crosses the building.
  v = evaluate_visibility(facing(-6, -15, 4, M_PI / 2), {4, -3, 0.9}, c, occ);
  EXPECT_TRUE(v.in_fov);
  EXPECT_TRUE(v.in_range);
  EXPECT_TRUE(v.occluded);
  EXPECT_FALSE(v.visible());

  // Behind the camera, and outside the horizontal FOV.
  EXPECT_FALSE(evaluate_visibility(south, {0, -20, 0.9}, c, occ).in_fov);
  EXPECT_FALSE(evaluate_visibility(south, {10, -14, 0.9}, c, occ).in_fov);
  // Straight below: outside the vertical FOV.
  EXPECT_FALSE(evaluate_visibility(south, {0, -14.5, 0.0}, c, occ).in_fov);
  // Too far.
  v = evaluate_visibility(facing(12, -40, 4, M_PI / 2), {12, -10, 0.9}, c, occ);
  EXPECT_TRUE(v.in_fov);
  EXPECT_FALSE(v.in_range);
}

TEST(Noise, CovarianceIsAnisotropicAlongLineOfSight)
{
  NoiseConfig n;
  const Vec3 los = Vec3(1, 1, 0).normalized();
  const Mat3 C = camera_noise_covariance(los, 10.0, n);
  const double sr = n.range_sigma_base_m + 10 * n.range_sigma_per_m;
  const double sc = n.cross_sigma_base_m + 10 * n.cross_sigma_per_m;
  EXPECT_NEAR(los.dot(C * los), sr * sr, 1e-12);
  const Vec3 across = Vec3(1, -1, 0).normalized();
  EXPECT_NEAR(across.dot(C * across), sc * sc, 1e-12);
  EXPECT_NEAR(C(2, 2), sc * sc, 1e-12);
  EXPECT_NEAR(across.dot(C * los), 0.0, 1e-12);
}

TEST(Synthesize, NavigationErrorLandsInTheDetection)
{
  CameraConfig c = ideal_camera();
  NoiseConfig quiet;
  quiet.range_sigma_base_m = quiet.cross_sigma_base_m = 1e-9;
  quiet.range_sigma_per_m = quiet.cross_sigma_per_m = 0.0;
  std::mt19937_64 rng(7);
  const Pose truth = facing(0, -15, 4, M_PI / 2);
  const Vec3 entity(0, -5, 0.9);

  // Perfect navigation: the detection is the entity.
  auto m = synthesize(truth, truth, {}, entity, c, quiet, rng);
  EXPECT_NEAR((m.position_world - entity).norm(), 0.0, 1e-6);

  // Position error: shifts the detection by the same amount.
  Pose est = truth;
  est.p += Vec3(0.3, -0.2, 0.1);
  m = synthesize(truth, est, {}, entity, c, quiet, rng);
  EXPECT_NEAR((m.position_world - (entity + Vec3(0.3, -0.2, 0.1))).norm(), 0.0, 1e-6);

  // Yaw error: rotates the relative vector about the agent.
  est = facing(0, -15, 4, M_PI / 2 + 0.01);
  m = synthesize(truth, est, {}, entity, c, quiet, rng);
  const double r = 10.0;  // horizontal distance to the entity
  EXPECT_NEAR(m.position_world.x(), -r * std::sin(0.01), 1e-6);
  EXPECT_NEAR(m.position_world.y(), -15 + r * std::cos(0.01), 1e-6);
}

TEST(Synthesize, SampleCovarianceMatchesReported)
{
  CameraConfig c = ideal_camera();
  NoiseConfig n;  // defaults
  NavUncertainty nav;
  nav.position_cov = Vec3(0.01, 0.02, 0.005).asDiagonal();
  std::mt19937_64 rng(42);
  const Pose truth = facing(3, -15, 4, M_PI / 2);
  const Vec3 entity(4, -2, 0.9);
  constexpr int kN = 40000;
  Vec3 mean = Vec3::Zero();
  Mat3 reported = Mat3::Zero();
  std::vector<Vec3> samples;
  samples.reserve(kN);
  for (int i = 0; i < kN; ++i) {
    const auto m = synthesize(truth, truth, nav, entity, c, n, rng);
    samples.push_back(m.position_world);
    mean += m.position_world;
    reported = m.covariance_world;
  }
  mean /= kN;
  Mat3 sample = Mat3::Zero();
  for (const auto & s : samples) {
    sample += (s - mean) * (s - mean).transpose();
  }
  sample /= (kN - 1);
  // The navigation position covariance is reported but not sampled here
  // (truth == estimate), so compare the sensor part only.
  const Mat3 sensor = reported - nav.position_cov;
  EXPECT_NEAR((mean - entity).norm(), 0.0, 0.01);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(sample(i, i), sensor(i, i), 0.05 * sensor(i, i)) << "axis " << i;
  }
  EXPECT_NEAR(sample(0, 1), sensor(0, 1), 0.05 * std::sqrt(sensor(0, 0) * sensor(1, 1)));
}

TEST(Synthesize, AttitudeUncertaintyGrowsWithLeverArm)
{
  CameraConfig c = ideal_camera();
  NoiseConfig n;
  std::mt19937_64 rng(1);
  NavUncertainty nav;
  nav.attitude_var = Vec3(0, 0, std::pow(0.5 * M_PI / 180, 2));  // 0.5 deg yaw sigma
  const Pose truth = facing(0, -20, 0.9, M_PI / 2);
  const Vec3 entity(0, 0, 0.9);  // 20 m straight ahead
  const auto with = synthesize(truth, truth, nav, entity, c, n, rng);
  const auto without = synthesize(truth, truth, {}, entity, c, n, rng);
  // Yaw error moves an entity 20 m ahead sideways (x here) by 20 * sigma_yaw.
  const double expected = std::pow(20.0 * 0.5 * M_PI / 180, 2);
  EXPECT_NEAR(with.covariance_world(0, 0) - without.covariance_world(0, 0), expected, 1e-9);
  EXPECT_NEAR(with.covariance_world(1, 1) - without.covariance_world(1, 1), 0.0, 1e-9);
}

TEST(SdfOccluders, StaticBoxesOnlyWithResolvedPoses)
{
  const std::string path = testing::TempDir() + "/occluders_test.sdf";
  std::ofstream(path) << R"(<?xml version="1.0"?>
<sdf version="1.9"><world name="w">
  <model name="ground"><static>true</static><link name="l"><collision name="c">
    <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
  </collision></link></model>
  <model name="block"><static>true</static><pose>1 2 3 0 0 1.5707963</pose>
    <link name="l"><pose>0.5 0 0 0 0 0</pose><collision name="c">
      <geometry><box><size>2 4 6</size></box></geometry></collision></link></model>
  <model name="cart"><link name="l"><collision name="c">
    <geometry><box><size>1 1 1</size></box></geometry></collision></link></model>
  <model name="entity"><static>true</static><link name="l"><collision name="c">
    <geometry><box><size>1 1 1</size></box></geometry></collision></link></model>
</world></sdf>)";
  const OccluderSet set = load_box_occluders(path, {}, {"entity"});
  ASSERT_EQ(set.boxes.size(), 1U);
  EXPECT_EQ(set.names[0], "block/l/c");
  ASSERT_EQ(set.skipped.size(), 1U);  // the plane
  // Link offset 0.5 m along the model's x, which points along world +y.
  EXPECT_NEAR(set.boxes[0].center.x(), 1.0, 1e-6);
  EXPECT_NEAR(set.boxes[0].center.y(), 2.5, 1e-6);
  EXPECT_NEAR(set.boxes[0].center.z(), 3.0, 1e-6);
  EXPECT_NEAR(set.boxes[0].half_extents.y(), 2.0, 1e-12);
  // Rotated: a ray along world x at y = 2.5 + 1.5 hits (the box is 4 m along world x).
  EXPECT_TRUE(segment_intersects({-5, 3.4, 3}, {5, 3.4, 3}, set.boxes[0]));
  EXPECT_FALSE(segment_intersects({-5, 3.6, 3}, {5, 3.6, 3}, set.boxes[0]));
  std::remove(path.c_str());
}

TEST(SdfOccluders, ProjectWorldHasTheBuilding)
{
  const OccluderSet set = load_box_occluders(COOP_WORLD_FILE, {COOP_MODEL_DIR}, {"entity"});
  ASSERT_EQ(set.boxes.size(), 1U);
  EXPECT_EQ(set.names[0], "building/link/collision");
  EXPECT_NEAR(set.boxes[0].half_extents.y(), 10.0, 1e-9);
  EXPECT_NEAR(set.boxes[0].center.z(), 3.0, 1e-9);
}
