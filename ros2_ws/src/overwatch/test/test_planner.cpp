#include <gtest/gtest.h>

#include <cmath>

#include "overwatch/planner.hpp"

using namespace overwatch;

namespace
{
synthetic_detector::Occluders wall()
{
  synthetic_detector::Occluders o;
  synthetic_detector::Obb b;  // 2 x 20 x 6 m, centred on the origin
  b.center = Vec3(0, 0, 3);
  b.half_extents = Vec3(1, 10, 3);
  o.boxes.push_back(b);
  return o;
}

PlannerConfig config()
{
  PlannerConfig c;
  c.radius_m = 14.0;
  c.interest_radius_m = 24.0;
  c.cell_m = 1.0;
  c.camera.hfov_rad = 90.0 * M_PI / 180.0;
  c.camera.vfov_rad = 70.0 * M_PI / 180.0;
  c.camera.pitch_down_rad = 40.0 * M_PI / 180.0;
  c.camera.max_range_m = 35.0;
  c.camera.min_range_m = 1.0;
  c.camera.mount_offset_body = Vec3::Zero();
  return c;
}
}  // namespace

TEST(Planner, HiddenCellsAreBehindTheWall)
{
  const Planner p(config(), wall());
  const auto cells = p.hidden_cells({-8, 0}, {});
  ASSERT_FALSE(cells.empty());
  for (const auto & c : cells) {
    // Hidden from (-8, 0): behind the wall, or just past its ends in its shadow;
    // never on the device's side of it.
    EXPECT_GE(c.point.x(), -1.0);
  }
  // A track in the hidden area weighs more.
  const std::vector<Vec2> track{{4, 0}};
  const auto weighted = p.hidden_cells({-8, 0}, track);
  double w0 = 0, w1 = 0;
  for (const auto & c : cells) {w0 += c.weight;}
  for (const auto & c : weighted) {w1 += c.weight;}
  EXPECT_GT(w1, w0);
}

TEST(Planner, OneAgentLooksBehindTheWall)
{
  const Planner p(config(), wall());
  const std::vector<double> alt{12.0};
  const std::vector<Vec3> pos{{-8, 0, 12}};
  const std::vector<std::optional<Spot>> cur(1);
  const auto spots = p.plan({-8, 0}, {}, alt, pos, cur);
  ASSERT_EQ(spots.size(), 1U);
  // The spot is over or beyond the wall, facing back towards the device, and
  // sees a good part of the hidden area.
  EXPECT_GT(spots[0].position.x(), -3.0);
  EXPECT_NEAR(spots[0].position.z(), 12.0, 1e-9);
  double total = 0;
  for (const auto & c : p.hidden_cells({-8, 0}, {})) {total += c.weight;}
  EXPECT_GT(spots[0].score, 0.2 * total);
}

TEST(Planner, TwoAgentsSpreadOutAndAddCoverage)
{
  const Planner p(config(), wall());
  const std::vector<double> alt{12.0, 14.0};
  const std::vector<Vec3> pos{{-8, -20, 12}, {-8, 20, 14}};
  const std::vector<std::optional<Spot>> cur(2);
  const auto spots = p.plan({-8, 0}, {}, alt, pos, cur);
  ASSERT_EQ(spots.size(), 2U);
  EXPECT_GE(std::hypot(spots[0].position.x() - spots[1].position.x(),
    spots[0].position.y() - spots[1].position.y()), config().min_separation_m);
  EXPECT_GE(spots[1].score, 0.0);
}

TEST(Planner, SmallDeviceMovesGiveSmallAgentMoves)
{
  const Planner p(config(), wall());
  const std::vector<double> alt{12.0};
  const std::vector<Vec3> pos{{-8, 0, 12}};
  const auto first = p.plan({-8, 0}, {}, alt, pos, std::vector<std::optional<Spot>>(1));
  // The device steps 1 m: the agent follows by a few metres at most.
  const std::vector<std::optional<Spot>> cur{first[0]};
  const auto second = p.plan({-8, 1}, {}, alt, pos, cur);
  EXPECT_LE(std::hypot(second[0].position.x() - first[0].position.x(),
    second[0].position.y() - first[0].position.y()), config().follow_radius_m + 1e-9);
}

TEST(Planner, StaysPutUnlessAJumpIsClearlyBetter)
{
  // Start from a spot that is decent but not the best: the planner keeps it
  // if it is within the hysteresis margin, and jumps when told to be greedy.
  PlannerConfig c = config();
  const Planner p(c, wall());
  const std::vector<double> alt{12.0};
  const std::vector<Vec3> pos{{-8, 0, 12}};
  const auto best = p.plan({-8, 0}, {}, alt, pos, std::vector<std::optional<Spot>>(1))[0];
  // The mirror image of the best spot, across the wall's axis: nearly as good.
  Spot mirror = best;
  mirror.position.y() = -best.position.y();
  const std::vector<std::optional<Spot>> cur{mirror};
  const auto kept = p.plan({-8, 0}, {}, alt, pos, cur)[0];
  EXPECT_NEAR(kept.position.y(), mirror.position.y(), 1e-9);
  c.hysteresis = 0.0;
  const Planner greedy(c, wall());
  const auto jumped = greedy.plan({-8, 0}, {}, alt, pos, cur)[0];
  EXPECT_GE(jumped.score, kept.score);
}
