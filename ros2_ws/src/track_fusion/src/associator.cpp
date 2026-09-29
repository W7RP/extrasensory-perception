#include "track_fusion/associator.hpp"

#include <algorithm>
#include <cmath>

namespace track_fusion
{

void NearestNeighbourAssociator::associate(const CostMatrix & c, Assignment & out) noexcept
{
  out.clear();
  const std::size_t pairs = std::min(c.tracks, c.detections);
  for (std::size_t k = 0; k < pairs; ++k) {
    double best = kGatedOut;
    int bi = kUnassigned;
    int bj = kUnassigned;
    for (std::size_t i = 0; i < c.tracks; ++i) {
      if (out.detection_for_track[i] != kUnassigned) {
        continue;
      }
      for (std::size_t j = 0; j < c.detections; ++j) {
        if (out.track_for_detection[j] == kUnassigned && c.cost[i][j] < best) {
          best = c.cost[i][j];
          bi = static_cast<int>(i);
          bj = static_cast<int>(j);
        }
      }
    }
    if (bi == kUnassigned) {
      return;  // nothing gated left
    }
    out.detection_for_track[static_cast<std::size_t>(bi)] = bj;
    out.track_for_detection[static_cast<std::size_t>(bj)] = bi;
  }
}

void HungarianAssociator::associate(const CostMatrix & c, Assignment & out) noexcept
{
  out.clear();
  const std::size_t rows = c.tracks;
  const std::size_t cols = c.detections;
  if (rows == 0 || cols == 0) {
    return;
  }
  // Square it up with padding; gated pairs get a cost larger than any sum of
  // real costs, so the optimum uses them only when it must, and they are then
  // discarded.
  constexpr std::size_t kN = std::max(kMaxTracks, kMaxDetections);
  const std::size_t n = std::max(rows, cols);
  double finite_max = 0.0;
  for (std::size_t i = 0; i < rows; ++i) {
    for (std::size_t j = 0; j < cols; ++j) {
      if (std::isfinite(c.cost[i][j])) {
        finite_max = std::max(finite_max, c.cost[i][j]);
      }
    }
  }
  const double big = (finite_max + 1.0) * static_cast<double>(n + 1);
  auto cost = [&](std::size_t i, std::size_t j) {
      if (i >= rows || j >= cols || !std::isfinite(c.cost[i][j])) {
        return big;
      }
      return c.cost[i][j];
    };

  // Jonker-Volgenant style shortest augmenting path formulation of the
  // Hungarian method (1-based arrays, as in the classic e-maxx presentation).
  std::array<double, kN + 1> u{};
  std::array<double, kN + 1> v{};
  std::array<std::size_t, kN + 1> p{};    // p[j]: row matched to column j
  std::array<std::size_t, kN + 1> way{};
  for (std::size_t i = 1; i <= n; ++i) {
    p[0] = i;
    std::size_t j0 = 0;
    std::array<double, kN + 1> minv;
    minv.fill(std::numeric_limits<double>::infinity());
    std::array<bool, kN + 1> used{};
    do {
      used[j0] = true;
      const std::size_t i0 = p[j0];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t j1 = 0;
      for (std::size_t j = 1; j <= n; ++j) {
        if (!used[j]) {
          const double cur = cost(i0 - 1, j - 1) - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }
      for (std::size_t j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const std::size_t j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }
  for (std::size_t j = 1; j <= n; ++j) {
    const std::size_t i = p[j];
    if (i == 0 || i > rows || j > cols || !std::isfinite(c.cost[i - 1][j - 1])) {
      continue;
    }
    out.detection_for_track[i - 1] = static_cast<int>(j - 1);
    out.track_for_detection[j - 1] = static_cast<int>(i - 1);
  }
}

std::unique_ptr<Associator> make_associator(std::string_view name)
{
  if (name == "nearest_neighbour") {
    return std::make_unique<NearestNeighbourAssociator>();
  }
  if (name == "hungarian") {
    return std::make_unique<HungarianAssociator>();
  }
  return nullptr;
}

}  // namespace track_fusion
