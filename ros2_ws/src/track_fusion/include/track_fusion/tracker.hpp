// Multi-agent track fusion: detections from any number of agents in, one set
// of tracks out.
//
// Per batch (one agent's frame, in time order):
//   1. predict every track to the batch time (covariance grows with time);
//   2. gate: squared Mahalanobis distance d2 of each detection from each
//      track's predicted position, with S = P_pos + R_detection; pairs beyond
//      the gate or of different classes are excluded; admitted pairs cost
//      d2 + ln|S| (the negative log likelihood, up to a constant);
//   3. associate with the configured Associator (swappable), in two passes:
//      established tracks first, then tentative tracks with what is left;
//   4. update each paired track (Kalman), and record which agent contributed;
//   5. birth: every unpaired detection starts a tentative track;
//   6. lifecycle: tentative -> confirmed after `confirm_hits` detections;
//      confirmed -> coasting after `coast_after_s` without one; a track is
//      deleted when it has gone without a detection too long for its status,
//      or its position uncertainty exceeds `max_position_sigma_m`.
// Batches must arrive in non-decreasing time order; the node's reorder
// buffer guarantees that, and older batches are refused (and counted).
//
// Real-time properties: fixed-capacity storage (types.hpp), no allocation,
// every step bounded by kMaxTracks x kMaxDetections. When the track table is
// full, births are refused and counted rather than evicting anything.
#pragma once

#include <cstdint>
#include <memory>

#include "track_fusion/associator.hpp"
#include "track_fusion/types.hpp"

namespace track_fusion
{

struct TrackerConfig
{
  double process_noise{0.5};          // q, white-acceleration density [m^2/s^3]
  double gate_d2{16.0};               // chi-square gate, 3 dof (~99.9 %)
  double init_velocity_sigma{1.5};    // [m/s] at birth: a walking-speed prior
  std::uint32_t confirm_hits{3};
  double tentative_timeout_s{1.0};    // tentative track dropped after this long unseen
  double coast_after_s{0.5};          // confirmed -> coasting
  double delete_after_s{10.0};        // coasting track dropped after this long unseen
  double max_position_sigma_m{5.0};   // or once its horizontal 1-sigma exceeds this
  double contribution_window_s{0.5};  // agent counts as "seeing" a track this long
};

struct TrackerCounters
{
  std::uint64_t batches{0};
  std::uint64_t detections{0};
  std::uint64_t associated{0};
  std::uint64_t births{0};
  std::uint64_t births_refused{0};    // track table full
  std::uint64_t deletions{0};
  std::uint64_t confirmations{0};
  std::uint64_t out_of_order{0};      // batches older than the tracker's time
};

// Track `in` predicted to time t (a copy). What the node publishes: every
// track at one common time.
[[nodiscard]] Track predicted(const Track & in, double t, double process_noise) noexcept;

// Bit (n-1) set when agent n contributed within `window_s` before t.
[[nodiscard]] std::uint8_t source_mask(const Track & tr, double t, double window_s) noexcept;

// Largest 1-sigma of the track's horizontal position ellipse [m].
[[nodiscard]] double horizontal_sigma(const Mat6 & P) noexcept;

class Tracker
{
public:
  Tracker(const TrackerConfig & cfg, std::unique_ptr<Associator> associator) noexcept;

  // Returns false (and changes nothing) for a batch older than the newest
  // processed time.
  bool process(const DetectionBatch & batch) noexcept;

  // Advance the lifecycle to time t (status changes and deletions only; states
  // stay at their last update time). process() calls it for every batch, and
  // detectors publish a batch every frame even when empty, so tracks age out
  // on time.
  void maintain(double t) noexcept;

  [[nodiscard]] const TrackSet & tracks() const noexcept {return set_;}
  [[nodiscard]] const TrackerCounters & counters() const noexcept {return n_;}
  [[nodiscard]] const TrackerConfig & config() const noexcept {return cfg_;}
  [[nodiscard]] double time() const noexcept {return t_;}
  [[nodiscard]] std::string_view associator_name() const noexcept {return assoc_->name();}


private:
  void birth(const Detection & d, std::uint8_t agent, double t) noexcept;
  void remove(std::size_t i) noexcept;

  TrackerConfig cfg_;
  std::unique_ptr<Associator> assoc_;
  TrackSet set_;
  CostMatrix cost_;
  Assignment assignment_;
  TrackerCounters n_;
  double t_{-1e300};
  std::uint32_t next_id_{1};
};

}  // namespace track_fusion
