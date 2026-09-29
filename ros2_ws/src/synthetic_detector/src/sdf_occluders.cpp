#include "synthetic_detector/sdf_occluders.hpp"

#include <algorithm>
#include <stdexcept>

#include <gz/math/Pose3.hh>
#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/ParserConfig.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>
#include <sdf/World.hh>

namespace synthetic_detector
{

namespace
{
Pose to_pose(const gz::math::Pose3d & p)
{
  Pose out;
  out.p = Vec3(p.Pos().X(), p.Pos().Y(), p.Pos().Z());
  out.q = Quat(p.Rot().W(), p.Rot().X(), p.Rot().Y(), p.Rot().Z());
  return out;
}

// Resolve an element's pose relative to `frame` ("" = its own relative_to),
// with libsdformat's SemanticPose, so relative_to chains are handled. Frame
// graphs are model-scoped: a collision resolves only as far as its model.
template<typename T>
gz::math::Pose3d resolve(const T & element, const std::string & frame)
{
  gz::math::Pose3d pose;
  const auto errors = element.SemanticPose().Resolve(pose, frame);
  if (!errors.empty()) {
    throw std::runtime_error("cannot resolve pose: " + errors.front().Message());
  }
  return pose;
}
}  // namespace

OccluderSet load_occluders(
  const std::string & world_file, const std::vector<std::string> & model_paths,
  const std::vector<std::string> & exclude)
{
  sdf::ParserConfig config;
  for (const auto & path : model_paths) {
    config.AddURIPath("model://", path);
  }
  sdf::Root root;
  const sdf::Errors errors = root.Load(world_file, config);
  if (!errors.empty()) {
    throw std::runtime_error("cannot load " + world_file + ": " + errors.front().Message());
  }
  if (root.WorldCount() == 0) {
    throw std::runtime_error(world_file + " has no <world>");
  }
  const sdf::World * world = root.WorldByIndex(0);
  OccluderSet set;
  for (std::size_t m = 0; m < world->ModelCount(); ++m) {
    const sdf::Model * model = world->ModelByIndex(m);
    if (!model->Static() ||
      std::find(exclude.begin(), exclude.end(), model->Name()) != exclude.end())
    {
      continue;
    }
    // Top-level models are placed relative to the world.
    const gz::math::Pose3d world_T_model = resolve(*model, "");
    for (std::size_t l = 0; l < model->LinkCount(); ++l) {
      const sdf::Link * link = model->LinkByIndex(l);
      for (std::size_t c = 0; c < link->CollisionCount(); ++c) {
        const sdf::Collision * col = link->CollisionByIndex(c);
        const std::string name = model->Name() + "/" + link->Name() + "/" + col->Name();
        const Pose pose = to_pose(world_T_model * resolve(*col, "__model__"));
        const sdf::Geometry * geom = col->Geom();
        if (const sdf::Box * box = geom->BoxShape()) {
          Obb obb;
          obb.center = pose.p;
          obb.q = pose.q;
          const auto size = box->Size();
          obb.half_extents = 0.5 * Vec3(size.X(), size.Y(), size.Z());
          set.boxes.push_back(obb);
        } else if (const sdf::Cylinder * cyl = geom->CylinderShape()) {
          set.cylinders.push_back({pose.p, pose.q, cyl->Radius(), 0.5 * cyl->Length()});
        } else if (const sdf::Sphere * sph = geom->SphereShape()) {
          set.spheres.push_back({pose.p, sph->Radius()});
        } else {
          set.skipped.push_back(name);  // ground planes, meshes: not occluders here
          continue;
        }
        set.names.push_back(name);
      }
    }
  }
  return set;
}

}  // namespace synthetic_detector
