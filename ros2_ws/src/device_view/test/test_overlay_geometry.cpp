#include <gtest/gtest.h>

#include <cmath>

#include "device_view/overlay_geometry.hpp"

using namespace device_view;

namespace
{
PinholeCamera camera()
{
  PinholeCamera c;  // 640 x 480, 70 deg horizontal FOV, like the device's
  c.fx = c.fy = 320.0 / std::tan(0.5 * 1.2217);
  return c;
}
}  // namespace

TEST(Projection, GazeboCameraConvention)
{
  const PinholeCamera cam = camera();
  const CameraPose pose = camera_on_device({0, 0, 0}, 0.0, {0, 0, 1.6});  // facing +x (east)
  // Straight ahead at camera height: the image centre.
  auto p = project(cam, pose, {10, 0, 1.6});
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->x(), 320.0, 1e-9);
  EXPECT_NEAR(p->y(), 240.0, 1e-9);
  // Left (+y) is left in the image; up is up.
  p = project(cam, pose, {10, 1, 2.6});
  ASSERT_TRUE(p);
  EXPECT_LT(p->x(), 320.0);
  EXPECT_LT(p->y(), 240.0);
  EXPECT_NEAR(p->x(), 320.0 - cam.fx * 0.1, 1e-9);
  // Behind the camera: nothing.
  EXPECT_FALSE(project(cam, pose, {-1, 0, 1.6}));
}

TEST(Projection, DeviceYawTurnsTheView)
{
  const PinholeCamera cam = camera();
  const CameraPose north = camera_on_device({2, 3, 0}, M_PI / 2, {0.1, 0, 1.6});
  EXPECT_NEAR(north.p.x(), 2.0, 1e-12);
  EXPECT_NEAR(north.p.y(), 3.1, 1e-12);  // the mount's forward offset turns with the device
  const auto p = project(cam, north, {2, 13.1, 1.6});
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->x(), 320.0, 1e-9);
  EXPECT_FALSE(project(cam, north, {2, -5, 1.6}));
}

TEST(Rect, IouAndClip)
{
  const Rect a{0, 0, 10, 10};
  EXPECT_DOUBLE_EQ(iou(a, a), 1.0);
  EXPECT_DOUBLE_EQ(iou(a, Rect{5, 0, 15, 10}), 50.0 / 150.0);
  EXPECT_DOUBLE_EQ(iou(a, Rect{20, 20, 30, 30}), 0.0);
  const PinholeCamera cam = camera();
  EXPECT_FALSE(clip(Rect{-50, -50, -10, -10}, cam));
  const auto c = clip(Rect{-50, 100, 50, 200}, cam);
  ASSERT_TRUE(c);
  EXPECT_DOUBLE_EQ(c->x0, 0.0);
  EXPECT_DOUBLE_EQ(c->x1, 50.0);
}

TEST(Box, CornersStandOnTheGroundAndTurn)
{
  const auto c = box_corners({4, -6, 0}, M_PI / 2, {0.5, 0.3, 1.75});
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(c[static_cast<std::size_t>(i)].z(), 0.0, 1e-12);
    EXPECT_NEAR(c[static_cast<std::size_t>(i) + 4].z(), 1.75, 1e-12);
  }
  // Facing north: the 0.5 m length runs along y.
  double ymin = 1e9, ymax = -1e9, xmin = 1e9, xmax = -1e9;
  for (const auto & p : c) {
    ymin = std::min(ymin, p.y()); ymax = std::max(ymax, p.y());
    xmin = std::min(xmin, p.x()); xmax = std::max(xmax, p.x());
  }
  EXPECT_NEAR(ymax - ymin, 0.5, 1e-12);
  EXPECT_NEAR(xmax - xmin, 0.3, 1e-12);
}

TEST(Box, ProjectedHeightMatchesPinholeScale)
{
  const PinholeCamera cam = camera();
  const CameraPose pose = camera_on_device({0, 0, 0}, 0.0, {0, 0, 1.6});
  const auto b = project_box(cam, pose, box_corners({16, 0, 0}, 0.0, {0.01, 0.5, 1.75}));
  ASSERT_TRUE(b);
  // A 1.75 m tall box 16 m away is fy * 1.75 / 16 pixels tall.
  EXPECT_NEAR(b->bounds.y1 - b->bounds.y0, cam.fy * 1.75 / 16.0, 0.5);
  // Any corner behind the camera: no box.
  EXPECT_FALSE(project_box(cam, pose, box_corners({0.3, 0, 0}, 0.0, {1.0, 0.5, 1.75})));
}

TEST(Silhouette, FacesTheCameraAndHasTheRightHeight)
{
  const auto s = person_silhouette({10, 0, 0}, 1.75, {0, 0, 1.6});
  double zmax = 0.0;
  for (const auto & p : s.body) {
    EXPECT_NEAR(p.x(), 10.0, 1e-12);  // flat, perpendicular to the line of sight
    zmax = std::max(zmax, p.z());
  }
  for (const auto & p : s.head) {
    EXPECT_NEAR(p.x(), 10.0, 1e-12);
    zmax = std::max(zmax, p.z());
  }
  EXPECT_NEAR(zmax, 1.74, 0.01);
  // Seen from the north it lies along x instead.
  const auto t = person_silhouette({0, 0, 0}, 1.75, {0, 10, 1.6});
  for (const auto & p : t.body) {
    EXPECT_NEAR(p.y(), 0.0, 1e-12);
  }
}

TEST(Ellipse, TwoSigmaAxes)
{
  Mat2 cov;
  cov << 4.0, 0.0, 0.0, 0.25;  // sigma 2 m along x, 0.5 m along y
  const auto e = ground_ellipse({1, 2}, cov, 2.0, 0.0);
  double xmax = -1e9, ymax = -1e9;
  for (const auto & p : e) {
    xmax = std::max(xmax, p.x());
    ymax = std::max(ymax, p.y());
    EXPECT_DOUBLE_EQ(p.z(), 0.0);
  }
  EXPECT_NEAR(xmax, 1.0 + 4.0, 1e-9);
  EXPECT_NEAR(ymax, 2.0 + 1.0, 0.02);
}

TEST(Presence, StylesFollowVisibility)
{
  EXPECT_EQ(presence(false, 0), Presence::kVisibleToDevice);
  EXPECT_EQ(presence(false, 2), Presence::kVisibleToDevice);
  EXPECT_EQ(presence(true, 1), Presence::kSeenThroughWall);
  EXPECT_EQ(presence(true, 0), Presence::kCoasting);
  EXPECT_DOUBLE_EQ(freshness(0.0, 4.0, 0.25), 1.0);
  EXPECT_DOUBLE_EQ(freshness(2.0, 4.0, 0.25), 0.625);
  EXPECT_DOUBLE_EQ(freshness(10.0, 4.0, 0.25), 0.25);
}
