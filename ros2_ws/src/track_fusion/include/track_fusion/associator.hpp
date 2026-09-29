// Data association: which detection updates which track.
//
// The tracker fills a cost matrix (for each track/detection pair, the negative
// log likelihood of the detection given the track's prediction, shifted to be
// non-negative, or kGatedOut where the pair fails the gate or the classes
// differ) and hands it to an Associator. The
// associator only decides the pairing, so strategies are interchangeable: pick
// one by name with make_associator(), or add a class here.
//
// Implementations must be noexcept, allocation-free and bounded in time for
// the fixed matrix size (kMaxTracks x kMaxDetections).
#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <memory>
#include <string_view>

#include "track_fusion/types.hpp"

namespace track_fusion
{

inline constexpr double kGatedOut = std::numeric_limits<double>::infinity();

struct CostMatrix
{
  std::size_t tracks{0};
  std::size_t detections{0};
  std::array<std::array<double, kMaxDetections>, kMaxTracks> cost{};
};

inline constexpr int kUnassigned = -1;

struct Assignment
{
  // detection_for_track[i] = detection index for track i, or kUnassigned.
  std::array<int, kMaxTracks> detection_for_track{};
  // track_for_detection[j] = track index for detection j, or kUnassigned.
  std::array<int, kMaxDetections> track_for_detection{};
  void clear() noexcept
  {
    detection_for_track.fill(kUnassigned);
    track_for_detection.fill(kUnassigned);
  }
};

class Associator
{
public:
  virtual ~Associator() = default;
  // Pairs only (track, detection) entries with a finite cost; each track and
  // each detection is used at most once.
  virtual void associate(const CostMatrix & c, Assignment & out) noexcept = 0;
  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

// Greedy nearest neighbour (the "global" greedy variant): repeatedly take the
// cheapest remaining gated pair. O(T * D * min(T, D)); not optimal in crowded
// scenes, but simple and plenty for a handful of well-separated entities.
class NearestNeighbourAssociator final : public Associator
{
public:
  void associate(const CostMatrix & c, Assignment & out) noexcept override;
  [[nodiscard]] std::string_view name() const noexcept override {return "nearest_neighbour";}
};

// Optimal assignment (minimum total cost over gated pairs), Hungarian /
// Kuhn-Munkres on the fixed-size matrix, O(n^3) with n = max(T, D). Gated-out
// pairs get a large finite cost and are dropped from the result afterwards.
class HungarianAssociator final : public Associator
{
public:
  void associate(const CostMatrix & c, Assignment & out) noexcept override;
  [[nodiscard]] std::string_view name() const noexcept override {return "hungarian";}
};

// "nearest_neighbour" or "hungarian"; nullptr for an unknown name. Allocates
// once, at configuration time.
[[nodiscard]] std::unique_ptr<Associator> make_associator(std::string_view name);

}  // namespace track_fusion
