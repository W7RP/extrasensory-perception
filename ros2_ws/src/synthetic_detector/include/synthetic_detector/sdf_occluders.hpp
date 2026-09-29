// Occluders straight from the Gazebo world file: every box, cylinder and
// sphere collision of every static model, placed in the world frame. Reading the same file
// Gazebo simulates means the detector's occlusion can never drift from the
// world the agents actually fly in.
#pragma once

#include <string>
#include <vector>

#include "synthetic_detector/geometry.hpp"

namespace synthetic_detector
{

struct OccluderSet : Occluders
{
  std::vector<std::string> names;       // "<model>/<link>/<collision>" of every shape
  std::vector<std::string> skipped;     // static collisions of other shapes (planes, meshes)
};

// Throws std::runtime_error if the file cannot be parsed. `model_paths` are
// the directories model:// URIs resolve against (the world includes models).
// Models named in `exclude` (e.g. the entity itself) are ignored.
[[nodiscard]] OccluderSet load_occluders(
  const std::string & world_file, const std::vector<std::string> & model_paths,
  const std::vector<std::string> & exclude);

}  // namespace synthetic_detector
