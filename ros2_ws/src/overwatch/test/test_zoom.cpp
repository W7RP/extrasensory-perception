#include <gtest/gtest.h>

#include <map>
#include <vector>

#include "overwatch/zoom.hpp"

using overwatch::ZoomCandidate;
using overwatch::ZoomConfig;
using overwatch::ZoomScheduler;

TEST(Zoom, NothingToLookAt)
{
  ZoomScheduler z(ZoomConfig{});
  EXPECT_FALSE(z.choose(0.0, {}));
}

TEST(Zoom, DwellsThenCyclesThroughEveryone)
{
  ZoomConfig cfg;
  cfg.dwell_s = 2.0;
  ZoomScheduler z(cfg);
  const std::vector<ZoomCandidate> c{{1, {}, false, false}, {2, {}, false, false},
    {3, {}, false, false}};
  std::map<std::uint32_t, int> looks;
  std::uint32_t last = 0;
  int switches = 0;
  for (int i = 0; i < 120; ++i) {  // 24 s at 5 Hz
    const auto id = z.choose(0.2 * i, c);
    ASSERT_TRUE(id);
    ++looks[*id];
    if (*id != last) {
      ++switches;
      last = *id;
    }
  }
  EXPECT_EQ(looks.size(), 3u);
  for (const auto & [id, n] : looks) {
    EXPECT_NEAR(n, 40, 6) << "track " << id;  // shared evenly
  }
  EXPECT_NEAR(switches, 12, 1);  // one switch per 2 s dwell
}

TEST(Zoom, HiddenAndCoastingTracksComeFirst)
{
  ZoomScheduler z(ZoomConfig{});
  const std::vector<ZoomCandidate> c{{1, {}, false, false}, {2, {}, true, false},
    {3, {}, false, true}};
  EXPECT_EQ(*z.choose(0.0, c), 2);  // hidden from the device
  EXPECT_EQ(*z.choose(3.1, c), 3);  // then the one that went quiet
  EXPECT_EQ(*z.choose(6.2, c), 1);
}

TEST(Zoom, MovesOnWhenItsTrackDisappears)
{
  ZoomScheduler z(ZoomConfig{});
  EXPECT_EQ(*z.choose(0.0, std::vector<ZoomCandidate>{{7, {}, false, false}}), 7);
  EXPECT_EQ(*z.choose(0.5, std::vector<ZoomCandidate>{{8, {}, false, false}}), 8);
  EXPECT_FALSE(z.choose(1.0, {}));
}
