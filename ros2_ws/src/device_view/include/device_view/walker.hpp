// The driven device's motion: a walker on the ground plane that cannot pass
// through anything solid at body height.
//
// ROS-free and unit-tested. Obstacles come from the same occluder set the
// detectors use (the world file): boxes and cylinders whose extent reaches
// below `body_top_m` block walking; spheres (tree canopies) and anything
// higher up do not. The walker is a circle of `radius_m`; on contact it slides
// along the obstacle instead of stopping dead, like a game character.
#pragma once

#include "synthetic_detector/geometry.hpp"

namespace device_view
{

struct WalkerConfig
{
  double radius_m{0.35};
  double body_top_m{1.8};          // obstacles lower than this do not block (curbs...)
  double body_bottom_m{0.2};       // ...and nothing higher than this above the ground does not either
  double walk_speed_mps{1.6};
  double run_speed_mps{3.5};
  double turn_rate_rps{1.9};       // ~110 deg/s
  double xmin{-1e9};
  double xmax{1e9};
  double ymin{-1e9};
  double ymax{1e9};
};

struct WalkerState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Command in the walker's own frame: forward, left, yaw rate, each in [-1, 1]
// (a stick or keys), plus "run".
struct WalkCommand
{
  double forward{0.0};
  double left{0.0};
  double turn{0.0};
  bool run{false};
};

// True if a circle of the walker's radius at (x, y) overlaps an obstacle or
// leaves the map.
[[nodiscard]] bool blocked(
  double x, double y, const WalkerConfig & cfg,
  const synthetic_detector::Occluders & obstacles) noexcept;

// Advance by dt: turn, then move, sliding along obstacles (the move is tried
// whole, then along x only, then along y only). Never ends in a blocked spot.
[[nodiscard]] WalkerState step(
  const WalkerState & s, const WalkCommand & cmd, double dt, const WalkerConfig & cfg,
  const synthetic_detector::Occluders & obstacles) noexcept;

}  // namespace device_view
