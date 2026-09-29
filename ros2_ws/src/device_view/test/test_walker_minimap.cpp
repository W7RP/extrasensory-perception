#include <gtest/gtest.h>

#include <cmath>

#include "device_view/minimap.hpp"
#include "device_view/walker.hpp"

using namespace device_view;
using synthetic_detector::Occluders;
using synthetic_detector::Vec3;

namespace
{
Occluders wall_and_tree()
{
  Occluders o;
  synthetic_detector::Obb wall;  // 2 x 20 x 6 m, centred on the origin
  wall.center = Vec3(0, 0, 3);
  wall.half_extents = Vec3(1, 10, 3);
  o.boxes.push_back(wall);
  o.cylinders.push_back({Vec3(10, 0, 1.5), synthetic_detector::Quat::Identity(), 0.3, 1.5});
  o.spheres.push_back({Vec3(10, 0, 4.0), 1.5});  // canopy: overhead, does not block walking
  return o;
}
}  // namespace

TEST(Walker, WalksTurnsAndRuns)
{
  WalkerConfig cfg;
  const Occluders none;
  WalkerState s{0, 0, 0};
  s = step(s, {1, 0, 0, false}, 1.0, cfg, none);
  EXPECT_NEAR(s.x, cfg.walk_speed_mps, 1e-9);
  s = step(s, {0, 0, 1, false}, M_PI / 2 / cfg.turn_rate_rps, cfg, none);
  EXPECT_NEAR(s.yaw, M_PI / 2, 1e-9);
  const WalkerState r = step(s, {1, 0, 0, true}, 1.0, cfg, none);
  EXPECT_NEAR(r.y - s.y, cfg.run_speed_mps, 1e-9);
  // Diagonal is no faster than straight.
  const WalkerState d = step({0, 0, 0}, {1, 1, 0, false}, 1.0, cfg, none);
  EXPECT_NEAR(std::hypot(d.x, d.y), cfg.walk_speed_mps, 1e-9);
}

TEST(Walker, CannotWalkThroughWallsButSlidesAlongThem)
{
  WalkerConfig cfg;
  const Occluders o = wall_and_tree();
  // Walking east into the wall's west face stops at the face.
  WalkerState s{-3, 0, 0};
  for (int i = 0; i < 100; ++i) {
    s = step(s, {1, 0, 0, false}, 0.05, cfg, o);
  }
  EXPECT_LT(s.x, -1.0 - cfg.radius_m + 1e-9);
  EXPECT_GT(s.x, -1.0 - cfg.radius_m - 0.1);
  // Walking diagonally into it slides north along the face.
  const double y0 = s.y;
  for (int i = 0; i < 20; ++i) {
    s = step(s, {0.7, 0.7, 0, false}, 0.05, cfg, o);
  }
  EXPECT_GT(s.y, y0 + 0.5);
  EXPECT_LT(s.x, -1.0 - cfg.radius_m + 1e-9);
  // The trunk blocks, the canopy does not.
  EXPECT_TRUE(blocked(10.4, 0, cfg, o));
  EXPECT_FALSE(blocked(11.0, 0, cfg, o));
}

TEST(Walker, StaysOnTheMap)
{
  WalkerConfig cfg;
  cfg.xmin = -5;
  cfg.xmax = 5;
  cfg.ymin = -5;
  cfg.ymax = 5;
  WalkerState s{0, 0, 0};
  for (int i = 0; i < 200; ++i) {
    s = step(s, {1, 0, 0, true}, 0.05, cfg, {});
  }
  EXPECT_LE(s.x, 5 - cfg.radius_m);
}

TEST(Fog, WallHidesTheFarSideFromTheDevice)
{
  const Occluders o = wall_and_tree();
  MinimapConfig m;
  m.xmin = -20;
  m.xmax = 20;
  m.ymin = -20;
  m.ymax = 20;
  m.cell_m = 1.0;
  Viewer device;  // west of the wall, facing east, a wide eye-level camera
  device.body.p = Vec3(-12, 0, 0);
  device.camera.hfov_rad = 2.0;
  device.camera.vfov_rad = 1.5;
  device.camera.pitch_down_rad = 0.0;
  device.camera.max_range_m = 60;
  device.camera.mount_offset_body = Vec3(0, 0, 1.6);
  Viewer agent;  // south of the wall's end, 10 m up, looking north and down
  agent.body.p = Vec3(4, -16, 10);
  agent.body.q = synthetic_detector::Quat(Eigen::AngleAxisd(M_PI / 2, Vec3::UnitZ()));
  agent.camera.hfov_rad = 1.6;
  agent.camera.vfov_rad = 1.2;
  agent.camera.pitch_down_rad = 0.6;
  agent.camera.max_range_m = 30;
  agent.camera.mount_offset_body = Vec3::Zero();
  const std::vector<Viewer> agents{agent};
  const auto fog = fog_of_war(m, device, agents, o);
  auto at = [&](double x, double y) {
      const int i = static_cast<int>((x - m.xmin) / m.cell_m);
      const int j = static_cast<int>((y - m.ymin) / m.cell_m);
      return fog[static_cast<std::size_t>(j * 40 + i)];
    };
  EXPECT_EQ(at(-6.5, 0.5), CellView::kDevice);   // in front of the wall
  EXPECT_EQ(at(4.5, 0.5), CellView::kAgent);     // behind it: only the agent sees it
  EXPECT_EQ(at(-18.5, 18.5), CellView::kNobody); // behind the device, out of the agent's view
  const cv::Mat img = draw_minimap(300, m, o, fog, device, agents, {});
  EXPECT_EQ(img.rows, 300);
  EXPECT_EQ(img.cols, 300);
}
