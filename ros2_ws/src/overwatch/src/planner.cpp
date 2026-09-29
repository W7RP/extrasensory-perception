#include "overwatch/planner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace overwatch
{

namespace
{
synthetic_detector::Pose body(const Spot & s)
{
  synthetic_detector::Pose p;
  p.p = s.position;
  p.q = synthetic_detector::Quat(Eigen::AngleAxisd(s.yaw, Vec3::UnitZ()));
  return p;
}
}  // namespace

Planner::Planner(PlannerConfig cfg, synthetic_detector::Occluders occluders)
: cfg_(std::move(cfg)), occ_(std::move(occluders)) {}

bool Planner::sees(const Spot & s, const Vec3 & point) const noexcept
{
  return synthetic_detector::evaluate_visibility(body(s), point, cfg_.camera, occ_).visible();
}

std::vector<Cell> Planner::hidden_cells(const Vec2 & device_xy, std::span<const Vec2> tracks) const
{
  std::vector<Cell> cells;
  const Vec3 eye(device_xy.x(), device_xy.y(), cfg_.device_eye_height_m);
  const double r = cfg_.interest_radius_m;
  const double area = cfg_.cell_m * cfg_.cell_m;
  for (double y = device_xy.y() - r; y <= device_xy.y() + r; y += cfg_.cell_m) {
    for (double x = device_xy.x() - r; x <= device_xy.x() + r; x += cfg_.cell_m) {
      if (std::hypot(x - device_xy.x(), y - device_xy.y()) > r || x < cfg_.xmin || x > cfg_.xmax ||
        y < cfg_.ymin || y > cfg_.ymax)
      {
        continue;
      }
      const Vec3 p(x, y, cfg_.probe_height_m);
      if (synthetic_detector::occluded(p, p, occ_)) {
        continue;  // inside a building or a trunk: nobody stands there
      }
      if (!synthetic_detector::occluded(eye, p, occ_)) {
        continue;  // the device can see it itself
      }
      double w = area;
      for (const auto & t : tracks) {
        if (std::hypot(x - t.x(), y - t.y()) <= cfg_.track_radius_m) {
          w *= cfg_.track_weight;
          break;
        }
      }
      cells.push_back({p, w});
    }
  }
  return cells;
}

std::vector<Spot> Planner::plan(
  const Vec2 & device_xy, std::span<const Vec2> tracks, std::span<const double> altitudes,
  std::span<const Vec3> positions, std::span<const std::optional<Spot>> current) const
{
  const std::vector<Cell> cells = hidden_cells(device_xy, tracks);
  std::vector<bool> covered(cells.size(), false);
  std::vector<Spot> chosen;

  auto coverage = [&](const Spot & s, bool mark) {
      double sum = 0.0;
      for (std::size_t i = 0; i < cells.size(); ++i) {
        if (!covered[i] && sees(s, cells[i].point)) {
          sum += cells[i].weight;
          if (mark) {
            covered[i] = true;
          }
        }
      }
      return sum;
    };
  auto separated = [&](const Vec3 & p) {
      return std::all_of(chosen.begin(), chosen.end(), [&](const Spot & c) {
               return std::hypot(c.position.x() - p.x(), c.position.y() - p.y()) >= cfg_.min_separation_m;
             });
    };
  auto facing_device = [&](double x, double y) {
      return std::atan2(device_xy.y() - y, device_xy.x() - x);
    };

  for (std::size_t k = 0; k < altitudes.size(); ++k) {
    const Vec3 here = k < positions.size() ? positions[k] : Vec3::Zero();
    Spot best;
    double best_value = -1e18;
    for (const double ring : {0.6 * cfg_.radius_m, cfg_.radius_m}) {
      for (int i = 0; i < cfg_.candidates; ++i) {
        const double a = 2.0 * M_PI * i / cfg_.candidates;
        const double x = std::clamp(device_xy.x() + ring * std::cos(a), cfg_.xmin, cfg_.xmax);
        const double y = std::clamp(device_xy.y() + ring * std::sin(a), cfg_.ymin, cfg_.ymax);
        Spot s{Vec3(x, y, altitudes[k]), facing_device(x, y), 0.0};
        if (!separated(s.position)) {
          continue;
        }
        s.score = coverage(s, false);
        const double value = s.score - cfg_.travel_cost_m2_per_m *
          std::hypot(x - here.x(), y - here.y());
        if (value > best_value) {
          best_value = value;
          best = s;
        }
      }
    }
    // Hysteresis: follow small shifts of the best spot freely, but only jump
    // somewhere else if it is clearly better than staying.
    if (k < current.size() && current[k] && separated(current[k]->position) &&
      std::hypot(best.position.x() - current[k]->position.x(),
      best.position.y() - current[k]->position.y()) > cfg_.follow_radius_m)
    {
      Spot keep = *current[k];
      keep.yaw = facing_device(keep.position.x(), keep.position.y());
      keep.position.z() = altitudes[k];
      keep.score = coverage(keep, false);
      if (keep.score >= (1.0 - cfg_.hysteresis) * best.score) {
        best = keep;
      }
    }
    if (best_value <= -1e18) {
      best = Spot{Vec3(here.x(), here.y(), altitudes[k]), facing_device(here.x(), here.y()), 0.0};
    }
    coverage(best, true);
    chosen.push_back(best);
  }
  return chosen;
}

}  // namespace overwatch
