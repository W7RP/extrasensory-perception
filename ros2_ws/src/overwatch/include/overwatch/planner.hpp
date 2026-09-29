// Overwatch planning: where should each agent be so that, between them, they
// see what the device cannot?
//
// ROS-free and unit-tested. Once per cycle, given the device's position:
//   1. hidden cells: the map cells within `interest_radius_m` of the device
//      that the device has no line of sight to from its eye height, whichever
//      way it faces (it can turn), leaving out cells inside obstacles. Cells within `track_radius_m` of a track the
//      device holds weigh `track_weight` times more: keep watching people
//      already found;
//   2. candidate spots, per agent at its own altitude: on two rings around the
//      device (0.6 R and R, `candidates` azimuths each), camera facing the
//      device;
//   3. coverage of a spot: the weighted hidden cells its camera would see (the
//      detectors' own model: field of view, range, line of sight);
//   4. greedy assignment, agent by agent: the spot adding the most uncovered
//      weight, minus a small travel cost, at least `min_separation_m` from the
//      spots already chosen;
//   5. hysteresis: an agent follows its best spot as it shifts (within
//      `follow_radius_m`), but jumps somewhere else only if that is more than
//      `hysteresis` better than staying, so agents do not flip between spots.
// Scores are in m^2 of hidden ground covered.
#pragma once

#include <optional>
#include <span>
#include <vector>

#include <Eigen/Core>

#include "synthetic_detector/geometry.hpp"
#include "synthetic_detector/sensor_model.hpp"

namespace overwatch
{

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

struct PlannerConfig
{
  double radius_m{14.0};
  double interest_radius_m{26.0};
  double cell_m{1.5};
  int candidates{24};
  double min_separation_m{8.0};
  double hysteresis{0.15};
  double follow_radius_m{3.0};
  double travel_cost_m2_per_m{0.3};
  double device_eye_height_m{1.6};
  double probe_height_m{0.9};
  double track_radius_m{3.0};
  double track_weight{3.0};
  double xmin{-1e9};
  double xmax{1e9};
  double ymin{-1e9};
  double ymax{1e9};
  synthetic_detector::CameraConfig camera;  // the agents' (all the same)
};

struct Spot
{
  Vec3 position{Vec3::Zero()};
  double yaw{0.0};     // camera heading, ENU
  double score{0.0};   // m^2 of hidden ground it covers (weighted)
};

struct Cell
{
  Vec3 point{Vec3::Zero()};
  double weight{0.0};
};

class Planner
{
public:
  Planner(PlannerConfig cfg, synthetic_detector::Occluders occluders);

  // Step 1, exposed for tests and displays.
  [[nodiscard]] std::vector<Cell> hidden_cells(
    const Vec2 & device_xy, std::span<const Vec2> tracks) const;

  // Spots for every agent. `altitudes[k]` is agent k's flight altitude;
  // `positions[k]` where it is now (for the travel cost); `current[k]` its
  // current spot, if any (for hysteresis).
  [[nodiscard]] std::vector<Spot> plan(
    const Vec2 & device_xy, std::span<const Vec2> tracks, std::span<const double> altitudes,
    std::span<const Vec3> positions, std::span<const std::optional<Spot>> current) const;

  // True if the device, at its eye height, has a line of sight to `point`.
  [[nodiscard]] bool device_sees(const Vec2 & device_xy, const Vec3 & point) const noexcept;

  [[nodiscard]] const PlannerConfig & config() const noexcept {return cfg_;}

private:
  [[nodiscard]] bool sees(const Spot & s, const Vec3 & point) const noexcept;

  PlannerConfig cfg_;
  synthetic_detector::Occluders occ_;
};

}  // namespace overwatch
