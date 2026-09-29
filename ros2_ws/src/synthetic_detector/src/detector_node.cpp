// Synthetic detector for one agent: stands in for camera + detector until a
// real one replaces it, and publishes exactly what a real one would.
//
//   in   /sim/ground_truth       true poses of this agent and the entity
//                                (simulation only: decides what the camera sees)
//        <ns>/eskf/odometry      the agent's own navigation estimate
//   out  <ns>/detections         coop_msgs/DetectionArray, one per frame,
//                                world frame, with covariance (the interface)
//        <ns>/visibility_truth   coop_msgs/VisibilityTruth (evaluation only)
//
// The interface contract a camera detector must keep to replace this node:
// publish one DetectionArray per processed frame on <ns>/detections, even when
// empty, stamped with the image's capture time (simulation clock), with world
// positions and full position covariances. The sensor model is documented in
// sensor_model.hpp.
//
// Timing: a sim-time timer at `rate_hz` is the camera's frame clock. Each
// frame uses the newest ground-truth sample and is stamped with that sample's
// time. The navigation estimate is the newest one (at most ~20 ms older: the
// ESKF publishes at 50 Hz), which is also what a live detector would have.

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <coop_msgs/msg/detection.hpp>
#include <coop_msgs/msg/detection_array.hpp>
#include <coop_msgs/msg/visibility_truth.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "synthetic_detector/sdf_occluders.hpp"
#include "synthetic_detector/sensor_model.hpp"

namespace synthetic_detector
{

namespace
{
Pose to_pose(const geometry_msgs::msg::Transform & t)
{
  Pose p;
  p.p = Vec3(t.translation.x, t.translation.y, t.translation.z);
  p.q = Quat(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z).normalized();
  return p;
}

double deg(double d) {return d * M_PI / 180.0;}
}  // namespace

class DetectorNode : public rclcpp::Node
{
public:
  DetectorNode()
  : Node("detector")
  {
    agent_id_ = static_cast<std::uint8_t>(declare_parameter<int>("agent_id", 1));
    agent_frame_ = declare_parameter<std::string>("agent_frame", "agent_1");
    entity_frame_ = declare_parameter<std::string>("entity_frame", "entity");
    entity_ref_height_ = declare_parameter<double>("entity_ref_height_m", 0.9);
    class_id_ = static_cast<std::uint8_t>(declare_parameter<int>("class_id",
      coop_msgs::msg::Detection::CLASS_PERSON));
    pose_source_ = declare_parameter<std::string>("pose_source", "eskf");
    if (pose_source_ != "eskf" && pose_source_ != "ground_truth") {
      throw std::invalid_argument("pose_source must be 'eskf' or 'ground_truth'");
    }
    const auto origin = declare_parameter<std::vector<double>>("origin_world_enu", {0.0, 0.0, 0.0});
    if (origin.size() != 3) {
      throw std::invalid_argument("origin_world_enu must have 3 values");
    }
    origin_ = Vec3(origin[0], origin[1], origin[2]);
    nav_timeout_s_ = declare_parameter<double>("nav_timeout_s", 0.5);
    const double rate_hz = declare_parameter<double>("rate_hz", 10.0);

    cam_.hfov_rad = deg(declare_parameter<double>("camera.hfov_deg", 90.0));
    cam_.vfov_rad = deg(declare_parameter<double>("camera.vfov_deg", 60.0));
    cam_.pitch_down_rad = deg(declare_parameter<double>("camera.pitch_down_deg", 15.0));
    const auto mount = declare_parameter<std::vector<double>>("camera.mount_offset_body",
        {0.12, 0.0, -0.05});
    cam_.mount_offset_body = Vec3(mount.at(0), mount.at(1), mount.at(2));
    cam_.min_range_m = declare_parameter<double>("camera.min_range_m", 0.5);
    cam_.max_range_m = declare_parameter<double>("camera.max_range_m", 25.0);
    noise_.range_sigma_base_m = declare_parameter<double>("noise.range_sigma_base_m", 0.15);
    noise_.range_sigma_per_m = declare_parameter<double>("noise.range_sigma_per_m", 0.02);
    noise_.cross_sigma_base_m = declare_parameter<double>("noise.cross_sigma_base_m", 0.05);
    noise_.cross_sigma_per_m = declare_parameter<double>("noise.cross_sigma_per_m", 0.005);
    noise_.p_miss = declare_parameter<double>("noise.p_miss", 0.1);
    rng_.seed(static_cast<std::uint64_t>(declare_parameter<int>("seed", 1)));

    const auto world_file = declare_parameter<std::string>("world_file", "");
    const auto model_paths = declare_parameter<std::vector<std::string>>("model_paths",
        std::vector<std::string>{});
    const auto exclude = declare_parameter<std::vector<std::string>>("exclude_models",
        {"entity"});
    if (world_file.empty()) {
      throw std::invalid_argument("world_file is required (the Gazebo world, for occluders)");
    }
    const OccluderSet occ = load_box_occluders(world_file, model_paths, exclude);
    occluders_ = occ.boxes;
    for (const auto & n : occ.names) {
      RCLCPP_INFO(get_logger(), "occluder: %s", n.c_str());
    }

    gt_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/sim/ground_truth", 10,
        [this](const tf2_msgs::msg::TFMessage & m) {on_ground_truth(m);});
    if (pose_source_ == "eskf") {
      nav_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("pose_topic", "eskf/odometry"), 10,
        [this](const nav_msgs::msg::Odometry & m) {on_nav(m);});
    }
    det_pub_ = create_publisher<coop_msgs::msg::DetectionArray>("detections", 10);
    vis_pub_ = create_publisher<coop_msgs::msg::VisibilityTruth>("visibility_truth", 10);
    timer_ = create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / rate_hz), [this]() {on_frame();});

    RCLCPP_INFO(get_logger(),
      "agent %u: %.0f Hz, pose from %s, %zu occluder(s), FOV %.0fx%.0f deg pitched %.0f deg, "
      "range %.1f-%.1f m, p_miss %.2f", agent_id_, rate_hz, pose_source_.c_str(),
      occluders_.size(), cam_.hfov_rad * 180 / M_PI, cam_.vfov_rad * 180 / M_PI,
      cam_.pitch_down_rad * 180 / M_PI, cam_.min_range_m, cam_.max_range_m, noise_.p_miss);
  }

private:
  void on_ground_truth(const tf2_msgs::msg::TFMessage & m)
  {
    for (const auto & t : m.transforms) {
      if (t.child_frame_id == agent_frame_) {
        agent_true_ = to_pose(t.transform);
        gt_stamp_ = t.header.stamp;
      } else if (t.child_frame_id == entity_frame_) {
        entity_true_ = to_pose(t.transform);
      }
    }
  }

  void on_nav(const nav_msgs::msg::Odometry & m)
  {
    // ESKF odometry: ENU position relative to where it initialised (this
    // agent's surveyed start point, origin_world_enu), FLU-to-ENU attitude.
    Pose p;
    p.p = origin_ + Vec3(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z);
    p.q = Quat(m.pose.pose.orientation.w, m.pose.pose.orientation.x,
      m.pose.pose.orientation.y, m.pose.pose.orientation.z).normalized();
    nav_pose_ = p;
    nav_.position_cov.setZero();
    for (int i = 0; i < 3; ++i) {
      nav_.position_cov(i, i) = m.pose.covariance[static_cast<std::size_t>(i * 7)];
      nav_.attitude_var[i] = m.pose.covariance[static_cast<std::size_t>(21 + i * 7)];
    }
    nav_stamp_ = m.header.stamp;
  }

  void on_frame()
  {
    if (!agent_true_ || !entity_true_) {
      return;  // no ground truth yet: the simulation is still starting
    }
    const rclcpp::Time stamp(gt_stamp_, RCL_ROS_TIME);
    if (stamp == last_frame_stamp_) {
      return;  // no new ground truth since the last frame (simulation paused)
    }
    last_frame_stamp_ = stamp;
    ++frames_;

    const Vec3 entity = entity_true_->p + Vec3(0.0, 0.0, entity_ref_height_);
    const Visibility vis = evaluate_visibility(*agent_true_, entity, cam_, occluders_);

    coop_msgs::msg::VisibilityTruth vt;
    vt.stamp = gt_stamp_;
    vt.agent = agent_id_;
    vt.entity = entity_frame_;
    vt.visible = vis.visible();
    vt.in_fov = vis.in_fov;
    vt.in_range = vis.in_range;
    vt.occluded = vis.occluded;
    vt.range_m = static_cast<float>(vis.range_m);
    vis_pub_->publish(vt);

    coop_msgs::msg::DetectionArray out;
    out.stamp = gt_stamp_;
    out.source_agent = agent_id_;
    out.frame_seq = frame_seq_++;

    // Where the agent believes it is. Without a current estimate a real
    // detector could not place anything in the world, so it reports nothing.
    std::optional<Pose> est;
    NavUncertainty nav;
    if (pose_source_ == "ground_truth") {
      est = agent_true_;
    } else if (nav_pose_ &&
      (stamp - rclcpp::Time(nav_stamp_, RCL_ROS_TIME)).seconds() < nav_timeout_s_)
    {
      est = nav_pose_;
      nav = nav_;
    } else {
      ++no_nav_frames_;
    }

    if (vis.visible()) {
      ++visible_frames_;
      std::bernoulli_distribution miss(noise_.p_miss);
      if (miss(rng_)) {
        ++dropped_;
      } else if (est) {
        const Measurement m = synthesize(*agent_true_, *est, nav, entity, cam_, noise_, rng_);
        coop_msgs::msg::Detection d;
        d.class_id = class_id_;
        d.confidence = static_cast<float>(m.confidence);
        for (int i = 0; i < 3; ++i) {
          d.position[static_cast<std::size_t>(i)] = static_cast<float>(m.position_world[i]);
        }
        const auto & C = m.covariance_world;
        d.covariance = {static_cast<float>(C(0, 0)), static_cast<float>(C(0, 1)),
          static_cast<float>(C(0, 2)), static_cast<float>(C(1, 1)),
          static_cast<float>(C(1, 2)), static_cast<float>(C(2, 2))};
        out.detections.push_back(d);
        ++detections_;
      }
    }
    det_pub_->publish(out);

    if (frames_ % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "frames %lu: entity visible in %lu, dropped %lu, detections %lu, frames without "
        "navigation %lu", static_cast<unsigned long>(frames_),
        static_cast<unsigned long>(visible_frames_), static_cast<unsigned long>(dropped_),
        static_cast<unsigned long>(detections_), static_cast<unsigned long>(no_nav_frames_));
    }
  }

  // Configuration
  std::uint8_t agent_id_{1};
  std::string agent_frame_;
  std::string entity_frame_;
  double entity_ref_height_{0.9};
  std::uint8_t class_id_{1};
  std::string pose_source_;
  Vec3 origin_{Vec3::Zero()};
  double nav_timeout_s_{0.5};
  CameraConfig cam_;
  NoiseConfig noise_;
  std::vector<Obb> occluders_;
  std::mt19937_64 rng_;

  // Latest inputs (single-threaded executor: no locking needed)
  std::optional<Pose> agent_true_;
  std::optional<Pose> entity_true_;
  builtin_interfaces::msg::Time gt_stamp_;
  std::optional<Pose> nav_pose_;
  NavUncertainty nav_;
  builtin_interfaces::msg::Time nav_stamp_;

  // Counters
  rclcpp::Time last_frame_stamp_{0, 0, RCL_ROS_TIME};
  std::uint32_t frame_seq_{0};
  std::uint64_t frames_{0};
  std::uint64_t visible_frames_{0};
  std::uint64_t dropped_{0};
  std::uint64_t detections_{0};
  std::uint64_t no_nav_frames_{0};

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr gt_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_sub_;
  rclcpp::Publisher<coop_msgs::msg::DetectionArray>::SharedPtr det_pub_;
  rclcpp::Publisher<coop_msgs::msg::VisibilityTruth>::SharedPtr vis_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace synthetic_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<synthetic_detector::DetectorNode>());
  rclcpp::shutdown();
  return 0;
}
