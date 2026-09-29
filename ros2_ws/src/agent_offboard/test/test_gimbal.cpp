#include <gtest/gtest.h>

#include <cmath>

#include "agent_offboard/gimbal.hpp"

using agent_offboard::GimbalAngles;
using agent_offboard::GimbalConfig;

TEST(Gimbal, PointsAtATarget)
{
  const GimbalConfig cfg;
  const auto g = agent_offboard::pointing_to({0, 0, 100}, {0, 100, 0}, cfg);
  EXPECT_NEAR(g.yaw, M_PI / 2, 1e-12);
  EXPECT_NEAR(g.pitch_down, M_PI / 4, 1e-12);
  // Straight below: pitch 90 deg; above the horizon: clamped to it.
  EXPECT_NEAR(agent_offboard::pointing_to({0, 0, 100}, {0, 0, 0}, cfg).pitch_down, M_PI / 2, 1e-12);
  EXPECT_NEAR(agent_offboard::pointing_to({0, 0, 0}, {10, 0, 5}, cfg).pitch_down, 0.0, 1e-12);
}

TEST(Gimbal, SlewsAtItsRateTheShortWayRound)
{
  GimbalConfig cfg;
  cfg.slew_rate_rad_s = 1.0;
  GimbalAngles cur{3.0, 1.0};
  const GimbalAngles cmd{-3.0, 0.5};  // 0.28 rad away across +-pi, not 6 rad
  const auto s = agent_offboard::step(cur, cmd, 0.1, cfg);
  EXPECT_NEAR(std::remainder(s.yaw - 3.1, 2 * M_PI), 0.0, 1e-12);
  EXPECT_NEAR(s.pitch_down, 0.9, 1e-12);
  for (int i = 0; i < 10; ++i) {
    cur = agent_offboard::step(cur, cmd, 0.1, cfg);
  }
  EXPECT_NEAR(agent_offboard::pointing_error(cur, cmd), 0.0, 1e-9);
}

TEST(Gimbal, OnTargetWithinAFractionOfTheField)
{
  GimbalConfig cfg;
  cfg.hfov_rad = 0.1;
  cfg.on_target_fraction = 0.2;  // 0.02 rad
  EXPECT_TRUE(agent_offboard::on_target({0.0, 0.5}, {0.0, 0.51}, cfg));
  EXPECT_FALSE(agent_offboard::on_target({0.0, 0.5}, {0.0, 0.53}, cfg));
  // Near straight down a large yaw difference is a small pointing error.
  EXPECT_TRUE(agent_offboard::on_target({0.0, M_PI / 2 - 0.005}, {1.0, M_PI / 2 - 0.005}, cfg));
}
