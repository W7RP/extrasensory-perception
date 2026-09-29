// Core types for track fusion. ROS-free and fixed-size: every container has a
// compile-time capacity, so nothing on the processing path allocates and the
// worst case (full arrays) bounds the cost of every call.
//
// Frame: the shared world frame (Gazebo ENU), metres, seconds. Time is
// simulation time in seconds (double: microsecond resolution for ~285 years).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <Eigen/Core>

namespace track_fusion
{

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

inline constexpr std::size_t kMaxDetections = 16;  // per batch (= coop_msgs bound)
inline constexpr std::size_t kMaxTracks = 32;      // = coop_msgs TrackArray bound
inline constexpr std::size_t kMaxAgents = 8;       // source_mask is 8 bits

struct Detection
{
  Vec3 z{Vec3::Zero()};       // world position [m]
  Mat3 R{Mat3::Identity()};   // its covariance [m^2]
  std::uint8_t class_id{0};
  float confidence{0.0F};
};

// Everything one agent reported in one frame.
struct DetectionBatch
{
  double t{0.0};
  std::uint8_t agent{0};      // 1-based
  std::uint32_t frame_seq{0};
  std::size_t count{0};
  std::array<Detection, kMaxDetections> detections{};
};

enum class TrackStatus : std::uint8_t
{
  kTentative = 0,
  kConfirmed = 1,
  kCoasting = 2,
};

struct Track
{
  std::uint32_t id{0};
  std::uint8_t class_id{0};
  TrackStatus status{TrackStatus::kTentative};
  Vec6 x{Vec6::Zero()};       // [px py pz vx vy vz]
  Mat6 P{Mat6::Identity()};
  double t{0.0};              // time the state refers to
  double born{0.0};
  double last_update{0.0};    // newest detection fused
  std::uint32_t hits{0};
  // Per-agent time of the newest detection it contributed (-inf = never).
  std::array<double, kMaxAgents> agent_last{};
};

// A fixed-capacity set of live tracks: slots [0, count) are in use.
struct TrackSet
{
  std::size_t count{0};
  std::array<Track, kMaxTracks> tracks{};
};

}  // namespace track_fusion
