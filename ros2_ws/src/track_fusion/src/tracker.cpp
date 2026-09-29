#include "track_fusion/tracker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "track_fusion/kalman.hpp"

namespace track_fusion
{

namespace
{
constexpr double kNever = -std::numeric_limits<double>::infinity();
}  // namespace

double horizontal_sigma(const Mat6 & P) noexcept
{
  const double a = P(0, 0);
  const double b = P(1, 1);
  const double c = P(0, 1);
  const double l1 = 0.5 * (a + b) + std::sqrt(0.25 * (a - b) * (a - b) + c * c);
  return std::sqrt(std::max(l1, 0.0));
}

Track predicted(const Track & in, double t, double process_noise) noexcept
{
  Track out = in;
  predict(out.x, out.P, t - out.t, process_noise);
  out.t = std::max(t, out.t);
  return out;
}

std::uint8_t source_mask(const Track & tr, double t, double window_s) noexcept
{
  std::uint8_t mask = 0;
  for (std::size_t a = 0; a < kMaxAgents; ++a) {
    if (t - tr.agent_last[a] <= window_s) {
      mask = static_cast<std::uint8_t>(mask | (1U << a));
    }
  }
  return mask;
}

Tracker::Tracker(const TrackerConfig & cfg, std::unique_ptr<Associator> associator) noexcept
: cfg_(cfg), assoc_(std::move(associator)) {}

bool Tracker::process(const DetectionBatch & batch) noexcept
{
  if (batch.t < t_) {
    ++n_.out_of_order;
    return false;
  }
  t_ = batch.t;
  ++n_.batches;
  n_.detections += batch.count;

  // 1. Predict everything to the batch time.
  for (std::size_t i = 0; i < set_.count; ++i) {
    Track & tr = set_.tracks[i];
    predict(tr.x, tr.P, batch.t - tr.t, cfg_.process_noise);
    tr.t = batch.t;
  }

  // 2.-4. Gate, associate and update, in two passes: established tracks
  // (confirmed or coasting) get first pick of the detections, tentative ones
  // share what is left. Without this, one outlier detection births a
  // tentative track whose large covariance makes the next detections look
  // "closer" to it than to the real track, which then starves: an ID switch.
  const std::size_t agent_slot = batch.agent >= 1 ? std::size_t{batch.agent} - 1 : 0;
  std::array<bool, kMaxDetections> used{};
  for (const bool established : {true, false}) {
    cost_.tracks = set_.count;
    cost_.detections = batch.count;
    for (std::size_t i = 0; i < set_.count; ++i) {
      const Track & tr = set_.tracks[i];
      const bool eligible = (tr.status != TrackStatus::kTentative) == established;
      for (std::size_t j = 0; j < batch.count; ++j) {
        const Detection & d = batch.detections[j];
        cost_.cost[i][j] = kGatedOut;
        if (!eligible || used[j] || d.class_id != tr.class_id) {
          continue;
        }
        // Gate on the Mahalanobis distance; rank by the negative log
        // likelihood (d2 + ln|S|), so a vague track does not win a detection
        // just because its wide covariance makes every distance look small.
        const Innovation in = innovation(tr.x, tr.P, d.z, d.R);
        if (in.d2 <= cfg_.gate_d2) {
          cost_.cost[i][j] = in.d2 + in.log_det_S;
        }
      }
    }
    // The likelihood cost can be negative; associators only compare costs,
    // except that kGatedOut marks exclusion. Shift into [0, inf) for them.
    double lowest = 0.0;
    for (std::size_t i = 0; i < set_.count; ++i) {
      for (std::size_t j = 0; j < batch.count; ++j) {
        lowest = std::min(lowest, cost_.cost[i][j]);
      }
    }
    for (std::size_t i = 0; i < set_.count; ++i) {
      for (std::size_t j = 0; j < batch.count; ++j) {
        cost_.cost[i][j] -= lowest;
      }
    }
    assoc_->associate(cost_, assignment_);
    for (std::size_t i = 0; i < set_.count; ++i) {
      const int j = assignment_.detection_for_track[i];
      if (j == kUnassigned) {
        continue;
      }
      used[static_cast<std::size_t>(j)] = true;
      Track & tr = set_.tracks[i];
      const Detection & d = batch.detections[static_cast<std::size_t>(j)];
      update(tr.x, tr.P, d.z, d.R);
      tr.last_update = batch.t;
      ++tr.hits;
      if (agent_slot < kMaxAgents) {
        tr.agent_last[agent_slot] = batch.t;
      }
      if (tr.status == TrackStatus::kTentative && tr.hits >= cfg_.confirm_hits) {
        tr.status = TrackStatus::kConfirmed;
        ++n_.confirmations;
      } else if (tr.status == TrackStatus::kCoasting) {
        tr.status = TrackStatus::kConfirmed;
      }
      ++n_.associated;
    }
  }

  // 5. Birth from what is left.
  for (std::size_t j = 0; j < batch.count; ++j) {
    if (!used[j]) {
      birth(batch.detections[j], batch.agent, batch.t);
    }
  }

  // 6. Lifecycle.
  maintain(batch.t);
  return true;
}

void Tracker::birth(const Detection & d, std::uint8_t agent, double t) noexcept
{
  if (set_.count == kMaxTracks) {
    ++n_.births_refused;
    return;
  }
  Track & tr = set_.tracks[set_.count++];
  tr = Track{};
  tr.id = next_id_++;
  tr.class_id = d.class_id;
  tr.status = cfg_.confirm_hits <= 1 ? TrackStatus::kConfirmed : TrackStatus::kTentative;
  tr.x.head<3>() = d.z;
  tr.x.tail<3>().setZero();
  tr.P.setZero();
  tr.P.block<3, 3>(0, 0) = d.R;
  tr.P.block<3, 3>(3, 3) = (cfg_.init_velocity_sigma * cfg_.init_velocity_sigma) *
    Mat3::Identity();
  tr.t = t;
  tr.born = t;
  tr.last_update = t;
  tr.hits = 1;
  tr.agent_last.fill(kNever);
  if (agent >= 1 && agent <= kMaxAgents) {
    tr.agent_last[agent - 1U] = t;
  }
  ++n_.births;
  if (tr.status == TrackStatus::kConfirmed) {
    ++n_.confirmations;
  }
}

void Tracker::remove(std::size_t i) noexcept
{
  // Order is not meaningful; move the last slot into the hole.
  set_.tracks[i] = set_.tracks[set_.count - 1];
  --set_.count;
  ++n_.deletions;
}

void Tracker::maintain(double t) noexcept
{
  std::size_t i = 0;
  while (i < set_.count) {
    Track & tr = set_.tracks[i];
    const double unseen = t - tr.last_update;
    bool drop = false;
    if (tr.status == TrackStatus::kTentative) {
      drop = unseen > cfg_.tentative_timeout_s;
    } else {
      if (tr.status == TrackStatus::kConfirmed && unseen > cfg_.coast_after_s) {
        tr.status = TrackStatus::kCoasting;
      }
      if (tr.status == TrackStatus::kCoasting) {
        // The uncertainty test uses the covariance predicted to t, without
        // changing the stored state.
        const Track p = predicted(tr, t, cfg_.process_noise);
        drop = unseen > cfg_.delete_after_s ||
          horizontal_sigma(p.P) > cfg_.max_position_sigma_m;
      }
    }
    if (drop) {
      remove(i);  // slot i now holds another track: check it without advancing
    } else {
      ++i;
    }
  }
}

}  // namespace track_fusion
