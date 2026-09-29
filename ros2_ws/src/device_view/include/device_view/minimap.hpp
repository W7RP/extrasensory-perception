// The game view's minimap: a north-up map of the scene as the device knows it.
//
// ROS-free (OpenCV for drawing). What it shows, and where each piece comes
// from, matters for honesty: the occluders are the map (the world file, which
// the device is assumed to have); the device and agent positions are their
// own reports (DeviceStatus, AgentStatus); the tracks are what the device
// fused. No ground truth.
//
// Fog of war: each map cell is shaded by who can see it right now: the
// device itself, only an agent, or nobody. "See" is the detectors' own model:
// field of view, range and a clear line of sight to the cell at person height.
#pragma once

#include <array>
#include <optional>

#include <cstdint>
#include <span>
#include <vector>

#include <opencv2/core.hpp>

#include "synthetic_detector/geometry.hpp"
#include "synthetic_detector/sensor_model.hpp"

namespace device_view
{

struct Viewer
{
  synthetic_detector::Pose body;             // world pose (ENU, FLU body)
  synthetic_detector::CameraConfig camera;
};

struct MapTrack
{
  double x{0.0};
  double y{0.0};
  std::uint32_t id{0};
  int agents_seeing{0};
  bool coasting{false};
  bool zoomed{false};   // the zoom camera is looking at it
};

struct MinimapConfig
{
  double xmin{-30.0};
  double xmax{30.0};
  double ymin{-30.0};
  double ymax{30.0};
  double cell_m{1.0};
  double probe_height_m{0.9};                // fog is computed for a person's centre
};

enum class CellView : std::uint8_t
{
  kNobody = 0,
  kAgent = 1,
  kDevice = 2,   // the device sees it (agents may too)
};

// The fog grid, row-major from (xmin, ymin): rows along y, columns along x.
[[nodiscard]] std::vector<CellView> fog_of_war(
  const MinimapConfig & cfg, const Viewer & device, std::span<const Viewer> agents,
  const synthetic_detector::Occluders & occ);

// Draw the whole minimap into a size x size image.
// A camera's field of view on the ground (z = 0): its four corner rays, when
// all of them meet the ground within its range (a camera looking steeply
// down, like a high agent's); nullopt otherwise (the minimap draws a wedge).
// Order: top left, top right, bottom right, bottom left as seen in the image.
[[nodiscard]] std::optional<std::array<synthetic_detector::Vec3, 4>> ground_footprint(const Viewer & v);

[[nodiscard]] cv::Mat draw_minimap(
  int size_px, const MinimapConfig & cfg, const synthetic_detector::Occluders & occ,
  const std::vector<CellView> & fog, const Viewer & device, std::span<const Viewer> agents,
  std::span<const MapTrack> tracks);

}  // namespace device_view
