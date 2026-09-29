// Simulation ground truth into ROS: world poses of the entity and every agent,
// straight from Gazebo's scene broadcaster, stamped with simulation time.
//
// SIMULATION AND EVALUATION ONLY. Consumers: each agent's synthetic detector
// (to decide what its camera could see) and the evaluation (the reference the
// tracks are scored against). Nothing on the fusion side subscribes to it.
//
// Why not ros_gz_bridge: its Pose_V -> tf2_msgs/TFMessage conversion takes
// frame names from per-pose header data that /world/<w>/pose/info does not
// carry, so every transform arrives unnamed (seen: empty child_frame_id). This
// node subscribes to the same Gazebo topic with gz-transport and keeps the
// names, renaming Gazebo models to project names on the way (x500_flow_1 ->
// agent_1).
//
// Output: /sim/ground_truth (tf2_msgs/TFMessage), frame_id "world", one
// transform per tracked model, published for every Gazebo pose message
// (~50 Hz), in the order given in `models`. The same transforms also go to
// /tf, so RViz can show where the entity and the agents really are, and the
// occluders read from the world file go to /sim/scene_markers (latched).

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "synthetic_detector/sdf_occluders.hpp"

namespace synthetic_detector
{

class GroundTruthBridge : public rclcpp::Node
{
public:
  GroundTruthBridge()
  : Node("ground_truth_bridge")
  {
    const auto world = declare_parameter<std::string>("world", "coop_field");
    models_ = declare_parameter<std::vector<std::string>>("models",
        {"entity", "x500_flow_1", "x500_flow_2"});
    names_ = declare_parameter<std::vector<std::string>>("names",
        {"entity", "agent_1", "agent_2"});
    if (models_.size() != names_.size() || models_.empty()) {
      throw std::invalid_argument("'models' and 'names' must be non-empty and the same length");
    }
    pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/sim/ground_truth", 10);
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    publish_scene(declare_parameter<std::string>("world_file", ""),
      declare_parameter<std::vector<std::string>>("model_paths", std::vector<std::string>{}),
      declare_parameter<std::vector<std::string>>("exclude_models", {"entity"}));
    msg_.transforms.resize(models_.size());
    for (std::size_t i = 0; i < names_.size(); ++i) {
      msg_.transforms[i].header.frame_id = "world";
      msg_.transforms[i].child_frame_id = names_[i];
    }
    topic_ = "/world/" + world + "/pose/info";
    if (!gz_.Subscribe(topic_, &GroundTruthBridge::on_poses, this)) {
      throw std::runtime_error("cannot subscribe to Gazebo topic " + topic_);
    }
    RCLCPP_INFO(get_logger(), "bridging %zu models from %s to %s", models_.size(),
      topic_.c_str(), pub_->get_topic_name());
  }

private:
  void publish_scene(
    const std::string & world_file, const std::vector<std::string> & model_paths,
    const std::vector<std::string> & exclude)
  {
    if (world_file.empty()) {
      return;
    }
    const OccluderSet occ = load_box_occluders(world_file, model_paths, exclude);
    visualization_msgs::msg::MarkerArray arr;
    for (std::size_t i = 0; i < occ.boxes.size(); ++i) {
      const Obb & b = occ.boxes[i];
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.ns = "occluders";
      m.id = static_cast<std::int32_t>(i);
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.pose.position.x = b.center.x();
      m.pose.position.y = b.center.y();
      m.pose.position.z = b.center.z();
      m.pose.orientation.w = b.q.w();
      m.pose.orientation.x = b.q.x();
      m.pose.orientation.y = b.q.y();
      m.pose.orientation.z = b.q.z();
      m.scale.x = 2.0 * b.half_extents.x();
      m.scale.y = 2.0 * b.half_extents.y();
      m.scale.z = 2.0 * b.half_extents.z();
      m.color.r = m.color.g = m.color.b = 0.6F;
      m.color.a = 0.6F;
      arr.markers.push_back(m);
    }
    scene_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/sim/scene_markers",
        rclcpp::QoS(1).reliable().transient_local());
    scene_pub_->publish(arr);
  }

  // Runs on a gz-transport thread. rclcpp publishers are thread-safe.
  void on_poses(const gz::msgs::Pose_V & in)
  {
    std::vector<bool> seen(models_.size(), false);
    for (const auto & p : in.pose()) {
      for (std::size_t i = 0; i < models_.size(); ++i) {
        // Model entries come first in pose/info; the first match is the model,
        // later same-named entries would be links.
        if (!seen[i] && p.name() == models_[i]) {
          auto & t = msg_.transforms[i];
          t.transform.translation.x = p.position().x();
          t.transform.translation.y = p.position().y();
          t.transform.translation.z = p.position().z();
          t.transform.rotation.w = p.orientation().w();
          t.transform.rotation.x = p.orientation().x();
          t.transform.rotation.y = p.orientation().y();
          t.transform.rotation.z = p.orientation().z();
          seen[i] = true;
        }
      }
    }
    for (std::size_t i = 0; i < models_.size(); ++i) {
      if (!seen[i]) {
        // A model that is not (yet) in the world: agents are spawned after
        // the world starts. Publish only complete sets.
        return;
      }
    }
    const auto & st = in.header().stamp();
    for (auto & t : msg_.transforms) {
      t.header.stamp.sec = static_cast<std::int32_t>(st.sec());
      t.header.stamp.nanosec = static_cast<std::uint32_t>(st.nsec());
    }
    pub_->publish(msg_);
    tf_pub_->publish(msg_);
  }

  std::vector<std::string> models_;
  std::vector<std::string> names_;
  std::string topic_;
  tf2_msgs::msg::TFMessage msg_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scene_pub_;
  gz::transport::Node gz_;
};

}  // namespace synthetic_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<synthetic_detector::GroundTruthBridge>());
  rclcpp::shutdown();
  return 0;
}
