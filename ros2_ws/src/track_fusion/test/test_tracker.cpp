#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include <Eigen/Cholesky>

#include "agent_estimation/alloc_probe.hpp"
#include "track_fusion/associator.hpp"
#include "track_fusion/kalman.hpp"
#include "track_fusion/reorder_buffer.hpp"
#include "track_fusion/tracker.hpp"

using namespace track_fusion;

namespace
{
constexpr std::uint8_t kPerson = 1;

DetectionBatch batch(double t, std::uint8_t agent, std::initializer_list<Vec3> zs,
  double sigma = 0.3)
{
  DetectionBatch b;
  b.t = t;
  b.agent = agent;
  for (const auto & z : zs) {
    Detection & d = b.detections[b.count++];
    d.z = z;
    d.R = (sigma * sigma) * Mat3::Identity();
    d.class_id = kPerson;
    d.confidence = 0.9F;
  }
  return b;
}

Tracker make_tracker(TrackerConfig cfg = {}, const char * assoc = "nearest_neighbour")
{
  return Tracker(cfg, make_associator(assoc));
}

std::size_t confirmed(const Tracker & t)
{
  std::size_t n = 0;
  for (std::size_t i = 0; i < t.tracks().count; ++i) {
    n += t.tracks().tracks[i].status != TrackStatus::kTentative;
  }
  return n;
}
}  // namespace

TEST(Kalman, PredictGrowsCovarianceUpdateShrinksIt)
{
  Vec6 x = Vec6::Zero();
  x[3] = 1.0;  // 1 m/s east
  Mat6 P = 0.01 * Mat6::Identity();
  predict(x, P, 2.0, 0.5);
  EXPECT_NEAR(x[0], 2.0, 1e-12);
  EXPECT_GT(P(0, 0), 0.01 + 2.0 * 2.0 * 0.01);  // velocity uncertainty feeds position
  const double before = P(0, 0);
  update(x, P, Vec3(2.2, 0, 0), 0.04 * Mat3::Identity());
  EXPECT_LT(P(0, 0), before);
  EXPECT_GT(x[0], 2.0);
  EXPECT_LT(x[0], 2.2);
  EXPECT_TRUE(P.isApprox(P.transpose()));
}

TEST(Kalman, ConsistentOnSimulatedConstantVelocityTarget)
{
  // NEES of the position estimate averages to ~3 (its dof) when the model is right.
  std::mt19937_64 rng(3);
  std::normal_distribution<double> n01(0.0, 1.0);
  constexpr double q = 0.5;
  constexpr double dt = 0.1;
  constexpr double sigma = 0.3;
  Vec6 truth = Vec6::Zero();
  truth[3] = 0.8;
  Vec6 x = Vec6::Zero();
  x.head<3>() = truth.head<3>();
  Mat6 P = Mat6::Identity();
  Mat6 F;
  Mat6 Q;
  cv_model(dt, q, F, Q);
  const Eigen::LLT<Mat6> Lq(Q);
  double nees = 0.0;
  int n = 0;
  for (int k = 0; k < 4000; ++k) {
    Vec6 w;
    for (int i = 0; i < 6; ++i) {w[i] = n01(rng);}
    truth = F * truth + Lq.matrixL() * w;
    predict(x, P, dt, q);
    const Vec3 z = truth.head<3>() + sigma * Vec3(n01(rng), n01(rng), n01(rng));
    update(x, P, z, sigma * sigma * Mat3::Identity());
    if (k > 100) {
      const Vec3 e = x.head<3>() - truth.head<3>();
      nees += e.dot(P.block<3, 3>(0, 0).ldlt().solve(e));
      ++n;
    }
  }
  EXPECT_NEAR(nees / n, 3.0, 0.3);
}

TEST(Associator, HungarianBeatsGreedyWhenGreedyIsWrong)
{
  // Greedy takes the single cheapest pair (0,0) = 1 and is left with (1,1) = 10:
  // total 11. The optimum crosses over: (0,1) + (1,0) = 2 + 3 = 5.
  CostMatrix c;
  c.tracks = 2;
  c.detections = 2;
  c.cost[0][0] = 1.0;
  c.cost[0][1] = 2.0;
  c.cost[1][0] = 3.0;
  c.cost[1][1] = 10.0;
  Assignment a;
  NearestNeighbourAssociator().associate(c, a);
  EXPECT_EQ(a.detection_for_track[0], 0);
  EXPECT_EQ(a.detection_for_track[1], 1);
  HungarianAssociator().associate(c, a);
  EXPECT_EQ(a.detection_for_track[0], 1);
  EXPECT_EQ(a.detection_for_track[1], 0);
  EXPECT_EQ(a.track_for_detection[0], 1);
}

TEST(Associator, GatedPairsAreNeverAssigned)
{
  for (const char * name : {"nearest_neighbour", "hungarian"}) {
    CostMatrix c;
    c.tracks = 3;
    c.detections = 2;
    for (auto & row : c.cost) {row.fill(kGatedOut);}
    c.cost[2][1] = 4.0;  // the only admissible pair
    Assignment a;
    make_associator(name)->associate(c, a);
    EXPECT_EQ(a.detection_for_track[0], kUnassigned) << name;
    EXPECT_EQ(a.detection_for_track[1], kUnassigned) << name;
    EXPECT_EQ(a.detection_for_track[2], 1) << name;
    EXPECT_EQ(a.track_for_detection[0], kUnassigned) << name;
  }
  EXPECT_EQ(make_associator("nope"), nullptr);
}

TEST(Associator, HungarianHandlesRectangularAndFullSize)
{
  // More detections than tracks, with a unique optimum along a shifted diagonal.
  CostMatrix c;
  c.tracks = kMaxTracks;
  c.detections = kMaxDetections;
  for (std::size_t i = 0; i < c.tracks; ++i) {
    for (std::size_t j = 0; j < c.detections; ++j) {
      c.cost[i][j] = (j == i % kMaxDetections && i < kMaxDetections) ? 0.5 : 5.0 + static_cast<double>(i + j);
    }
  }
  Assignment a;
  HungarianAssociator().associate(c, a);
  for (std::size_t i = 0; i < kMaxDetections; ++i) {
    EXPECT_EQ(a.detection_for_track[i], static_cast<int>(i));
  }
  for (std::size_t i = kMaxDetections; i < kMaxTracks; ++i) {
    EXPECT_EQ(a.detection_for_track[i], kUnassigned);
  }
}

TEST(ReorderBuffer, ReleasesInTimeOrderAtTheWatermark)
{
  ReorderBuffer rb(0.3);
  DetectionBatch ev;
  DetectionBatch out;
  EXPECT_FALSE(rb.push(batch(10.1, 2, {}), ev));
  // Only agent 2 heard from: the watermark is its newest stamp.
  ASSERT_TRUE(rb.pop_ready(out));
  EXPECT_DOUBLE_EQ(out.t, 10.1);
  rb.push(batch(10.2, 2, {}), ev);
  rb.push(batch(10.0, 1, {}), ev);   // agent 1 is behind
  // Watermark = min(agent1 10.0, agent2 10.2) = 10.0: only 10.0 is released.
  ASSERT_TRUE(rb.pop_ready(out));
  EXPECT_DOUBLE_EQ(out.t, 10.0);
  EXPECT_FALSE(rb.pop_ready(out));
  rb.push(batch(10.2, 1, {}), ev);
  ASSERT_TRUE(rb.pop_ready(out));
  EXPECT_DOUBLE_EQ(out.t, 10.2);
  ASSERT_TRUE(rb.pop_ready(out));
  EXPECT_DOUBLE_EQ(out.t, 10.2);
  // Agent 1 goes quiet: agent 2 is held back at most max_hold_s.
  rb.push(batch(10.3, 2, {}), ev);
  EXPECT_FALSE(rb.pop_ready(out));
  rb.push(batch(10.7, 2, {}), ev);
  ASSERT_TRUE(rb.pop_ready(out));
  EXPECT_DOUBLE_EQ(out.t, 10.3);
}

TEST(Tracker, BirthConfirmCoastAndDelete)
{
  TrackerConfig cfg;
  cfg.confirm_hits = 3;
  cfg.coast_after_s = 0.5;
  cfg.delete_after_s = 3.0;
  cfg.max_position_sigma_m = 100.0;  // delete on time, for this test
  Tracker tr = make_tracker(cfg);
  for (int k = 0; k < 2; ++k) {
    tr.process(batch(k * 0.1, 1, {{k * 0.1, 0, 0.9}}));
  }
  ASSERT_EQ(tr.tracks().count, 1U);
  EXPECT_EQ(tr.tracks().tracks[0].status, TrackStatus::kTentative);
  tr.process(batch(0.2, 1, {{0.2, 0, 0.9}}));
  EXPECT_EQ(tr.tracks().tracks[0].status, TrackStatus::kConfirmed);
  const auto id = tr.tracks().tracks[0].id;
  // Empty frames: the track coasts, then goes.
  tr.process(batch(0.9, 1, {}));
  EXPECT_EQ(tr.tracks().tracks[0].status, TrackStatus::kCoasting);
  tr.process(batch(3.1, 1, {}));
  ASSERT_EQ(tr.tracks().count, 1U);
  tr.process(batch(3.3, 1, {}));
  EXPECT_EQ(tr.tracks().count, 0U);
  // A new detection is a new track with a new id.
  tr.process(batch(3.4, 1, {{0, 0, 0.9}}));
  ASSERT_EQ(tr.tracks().count, 1U);
  EXPECT_NE(tr.tracks().tracks[0].id, id);
  EXPECT_EQ(tr.counters().deletions, 1U);
}

TEST(Tracker, TwoAgentsOneEntityMakeOneTrackFaster)
{
  // The same entity seen by both agents at 10 Hz: one track, confirmed after
  // two frames (four detections) instead of three.
  Tracker both = make_tracker();
  Tracker one = make_tracker();
  double t_both = -1;
  double t_one = -1;
  for (int k = 0; k < 10; ++k) {
    const double t = k * 0.1;
    const Vec3 p(4, -6 + 0.08 * k, 0.9);
    both.process(batch(t, 1, {p + Vec3(0.2, 0, 0)}));
    both.process(batch(t, 2, {p - Vec3(0, 0.2, 0)}));
    one.process(batch(t, 1, {p + Vec3(0.2, 0, 0)}));
    if (t_both < 0 && confirmed(both)) {t_both = t;}
    if (t_one < 0 && confirmed(one)) {t_one = t;}
  }
  EXPECT_EQ(both.tracks().count, 1U);
  EXPECT_EQ(both.tracks().tracks[0].hits, 20U);
  EXPECT_EQ(source_mask(both.tracks().tracks[0], 0.9, 0.5), 0b11);
  EXPECT_EQ(source_mask(one.tracks().tracks[0], 0.9, 0.5), 0b01);
  EXPECT_LT(t_both, t_one);
  // Fused covariance is smaller than single-agent.
  EXPECT_LT(both.tracks().tracks[0].P(0, 0), one.tracks().tracks[0].P(0, 0));
}

TEST(Tracker, SeparateEntitiesStaySeparateAndStrangersAreGated)
{
  Tracker tr = make_tracker();
  for (int k = 0; k < 5; ++k) {
    tr.process(batch(k * 0.1, 1, {{0, 0, 0.9}, {6, 0, 0.9}}));
  }
  EXPECT_EQ(tr.tracks().count, 2U);
  EXPECT_EQ(confirmed(tr), 2U);
  // A detection 3 m from both (far outside a 0.3 m-sigma gate) starts its own track.
  tr.process(batch(0.5, 1, {{3, 0, 0.9}}));
  EXPECT_EQ(tr.tracks().count, 3U);
}

TEST(Tracker, OutlierDoesNotStealTheTrack)
{
  // A confirmed track, one 4.5-sigma outlier (births a tentative track), then
  // detections back on the entity: they must keep going to the original track.
  Tracker tr = make_tracker();
  for (int k = 0; k < 10; ++k) {
    tr.process(batch(k * 0.1, 1, {{0, 0, 0.9}}));
  }
  tr.process(batch(1.0, 1, {{0, 1.6, 0.9}}));  // outside the gate: a new tentative track
  ASSERT_EQ(tr.tracks().count, 2U);
  const std::uint32_t original = tr.tracks().tracks[0].id;
  for (int k = 11; k < 22; ++k) {
    tr.process(batch(k * 0.1, 1, {{0, 0.4, 0.9}}));  // between the two, nearer the original
  }
  ASSERT_EQ(tr.tracks().count, 1U);  // the tentative one aged out
  EXPECT_EQ(tr.tracks().tracks[0].id, original);
  EXPECT_EQ(tr.tracks().tracks[0].hits, 21U);
}

TEST(Tracker, RefusesOutOfOrderBatches)
{
  Tracker tr = make_tracker();
  EXPECT_TRUE(tr.process(batch(1.0, 1, {{0, 0, 0.9}})));
  EXPECT_FALSE(tr.process(batch(0.9, 2, {{0, 0, 0.9}})));
  EXPECT_EQ(tr.counters().out_of_order, 1U);
  EXPECT_TRUE(tr.process(batch(1.0, 2, {{0, 0, 0.9}})));  // equal time is fine
}

TEST(Tracker, UncertaintyGrowsWhileCoastingUntilDeleted)
{
  TrackerConfig cfg;
  cfg.delete_after_s = 1000.0;
  cfg.max_position_sigma_m = 2.0;
  Tracker tr = make_tracker(cfg);
  for (int k = 0; k < 5; ++k) {
    tr.process(batch(k * 0.1, 1, {{0, 0, 0.9}}));
  }
  const Track t0 = tr.tracks().tracks[0];
  const Track later = predicted(t0, 3.0, cfg.process_noise);
  EXPECT_GT(horizontal_sigma(later.P), 2.0 * horizontal_sigma(t0.P));
  double t = 0.5;
  while (tr.tracks().count > 0 && t < 100.0) {
    tr.process(batch(t, 1, {}));
    t += 0.1;
  }
  EXPECT_EQ(tr.tracks().count, 0U);
  // Deleted on the uncertainty limit, well before delete_after_s.
  EXPECT_LT(t, 30.0);
}

TEST(Tracker, FullTableRefusesBirthsInsteadOfEvicting)
{
  Tracker tr = make_tracker();
  DetectionBatch b;
  b.t = 0.0;
  b.agent = 1;
  for (std::size_t round = 0; round < 3; ++round) {
    b.count = 0;
    for (std::size_t j = 0; j < kMaxDetections; ++j) {
      Detection & d = b.detections[b.count++];
      d.z = Vec3(10.0 * static_cast<double>(round * kMaxDetections + j), 0, 0.9);
      d.R = 0.01 * Mat3::Identity();
      d.class_id = kPerson;
    }
    tr.process(b);
  }
  EXPECT_EQ(tr.tracks().count, kMaxTracks);
  EXPECT_EQ(tr.counters().births_refused, 3 * kMaxDetections - kMaxTracks);
}

TEST(Tracker, ProcessNeverAllocates)
{
  // This test executable links agent_estimation's alloc_probe, which counts
  // every operator new made inside a HotPathScope.
  for (const char * name : {"nearest_neighbour", "hungarian"}) {
    Tracker tr = make_tracker({}, name);
    std::mt19937_64 rng(5);
    std::normal_distribution<double> n(0.0, 0.3);
    const auto before = agent_estimation::alloc_probe::hot_path_allocations();
    {
      agent_estimation::alloc_probe::HotPathScope hot;
      for (int k = 0; k < 500; ++k) {
        const double t = k * 0.05;
        const std::uint8_t agent = static_cast<std::uint8_t>(1 + k % 2);
        tr.process(batch(t, agent, {{0.5 * t + n(rng), n(rng), 0.9}, {n(rng), 5 + n(rng), 0.9}}));
      }
    }
    EXPECT_EQ(agent_estimation::alloc_probe::hot_path_allocations(), before) << name;
    // Two established tracks; a rare >4-sigma outlier may leave a short-lived
    // tentative one, which must never displace them.
    EXPECT_EQ(confirmed(tr), 2U) << name;
    EXPECT_LE(tr.counters().births, 2U + 5U) << name;
  }
}
