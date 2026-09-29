// The ground device's body: moves it when it is driven, and tells the agents
// where it is.
//
//   in   ~/cmd  (geometry_msgs/Twist)  when `mode: teleop`: linear.x forward,
//                                      linear.y left, angular.z turn, each in
//                                      [-1, 1]; linear.z > 0.5 = run
//        /sim/ground_truth             the device's pose (Phase 2+ assumes a
//                                      well-localised device)
//   out  /device/status (coop_msgs/DeviceStatus, 10 Hz)  its pose, for the
//                                      agents' planner and the game view
//
// In teleop mode it integrates the command with the walker model (walker.hpp:
// walk/run speeds, turn rate, sliding collision against everything solid at
// body height in the world file) on a simulation-time timer, and moves the
// static Gazebo model with an in-process set_pose request. A command older
// than `cmd_timeout_s` counts as "stop". In scripted mode (the scenario's
// route, moved by scene_mover.py) it only publishes the status.

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

#include <coop_msgs/msg/device_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "device_view/walker.hpp"
#include "synthetic_detector/sdf_occluders.hpp"

namespace device_view
{

class DeviceController : public rclcpp::Node
{
public:
  DeviceController()
  : Node("device_controller")
  {
    mode_ = declare_parameter<std::string>("mode", "scripted");
    if (mode_ != "scripted" && mode_ != "teleop") {
      throw std::invalid_argument("mode must be 'scripted' or 'teleop'");
    }
    world_ = declare_parameter<std::string>("world", "coop_field");
    model_ = declare_parameter<std::string>("model", "device");
    frame_ = declare_parameter<std::string>("device_frame", "device");
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.3);
    cfg_.radius_m = declare_parameter<double>("radius_m", cfg_.radius_m);
    cfg_.walk_speed_mps = declare_parameter<double>("walk_speed_mps", cfg_.walk_speed_mps);
    cfg_.run_speed_mps = declare_parameter<double>("run_speed_mps", cfg_.run_speed_mps);
    cfg_.turn_rate_rps = declare_parameter<double>("turn_rate_dps", 110.0) * M_PI / 180.0;
    const auto bounds = declare_parameter<std::vector<double>>("bounds_xy", {-1e9, 1e9, -1e9, 1e9});
    if (bounds.size() == 4) {
      cfg_.xmin = bounds[0];
      cfg_.xmax = bounds[1];
      cfg_.ymin = bounds[2];
      cfg_.ymax = bounds[3];
    }
    const auto world_file = declare_parameter<std::string>("world_file", "");
    if (mode_ == "teleop") {
      if (world_file.empty()) {
        throw std::invalid_argument("world_file is required in teleop mode (obstacles)");
      }
      obstacles_ = synthetic_detector::load_occluders(world_file,
          declare_parameter<std::vector<std::string>>("model_paths", std::vector<std::string>{}),
          declare_parameter<std::vector<std::string>>("exclude_models", {"entity"}));
      cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("~/cmd", 10,
          [this](const geometry_msgs::msg::Twist & m) {
            cmd_.forward = m.linear.x;
            cmd_.left = m.linear.y;
            cmd_.turn = m.angular.z;
            cmd_.run = m.linear.z > 0.5;
            last_cmd_ = now();
          });
      move_timer_ = create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / 60.0),
          [this]() {on_move();});
    }
    gt_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/sim/ground_truth", 10,
        [this](const tf2_msgs::msg::TFMessage & m) {on_ground_truth(m);});
    status_pub_ = create_publisher<coop_msgs::msg::DeviceStatus>("/device/status", 10);
    status_timer_ = create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.1),
        [this]() {publish_status();});
    RCLCPP_INFO(get_logger(), "device %s: %s%s", model_.c_str(), mode_.c_str(),
      mode_ == "teleop" ? ", driven from ~/cmd" : ", moved by the scenario");
  }

private:
  void on_ground_truth(const tf2_msgs::msg::TFMessage & m)
  {
    for (const auto & tf : m.transforms) {
      if (tf.child_frame_id != frame_) {
        continue;
      }
      const auto & q = tf.transform.rotation;
      truth_ = WalkerState{tf.transform.translation.x, tf.transform.translation.y,
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
      if (!state_) {
        state_ = truth_;  // start from wherever the device stands in the world
      }
    }
  }

  void on_move()
  {
    const rclcpp::Time t = now();
    if (!state_ || t.nanoseconds() == 0) {
      return;
    }
    const double dt = last_move_.nanoseconds() > 0 ? (t - last_move_).seconds() : 0.0;
    last_move_ = t;
    if (dt <= 0.0 || dt > 0.5) {
      return;
    }
    WalkCommand cmd = cmd_;
    if (last_cmd_.nanoseconds() == 0 || (t - last_cmd_).seconds() > cmd_timeout_s_) {
      cmd = WalkCommand{};
    }
    if (cmd.forward == 0.0 && cmd.left == 0.0 && cmd.turn == 0.0) {
      return;
    }
    state_ = step(*state_, cmd, dt, cfg_, obstacles_);
    gz::msgs::Pose req;
    req.set_name(model_);
    req.mutable_position()->set_x(state_->x);
    req.mutable_position()->set_y(state_->y);
    req.mutable_position()->set_z(0.0);
    req.mutable_orientation()->set_w(std::cos(0.5 * state_->yaw));
    req.mutable_orientation()->set_z(std::sin(0.5 * state_->yaw));
    gz::msgs::Boolean rep;
    bool ok = false;
    if (!gz_.Request("/world/" + world_ + "/set_pose", req, 200, rep, ok) || !ok) {
      if (++failures_ % 60 == 1) {
        RCLCPP_WARN(get_logger(), "set_pose on %s failed (%lu so far)", model_.c_str(),
          static_cast<unsigned long>(failures_));
      }
    }
  }

  void publish_status()
  {
    if (!truth_) {
      return;
    }
    coop_msgs::msg::DeviceStatus s;
    s.stamp = now();
    s.position = {static_cast<float>(truth_->x), static_cast<float>(truth_->y), 0.0F};
    s.yaw = static_cast<float>(truth_->yaw);
    status_pub_->publish(s);
  }

  std::string mode_;
  std::string world_;
  std::string model_;
  std::string frame_;
  double cmd_timeout_s_{0.3};
  WalkerConfig cfg_;
  synthetic_detector::Occluders obstacles_;

  WalkCommand cmd_;
  rclcpp::Time last_cmd_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_move_{0, 0, RCL_ROS_TIME};
  std::optional<WalkerState> state_;
  std::optional<WalkerState> truth_;
  std::uint64_t failures_{0};

  gz::transport::Node gz_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr gt_sub_;
  rclcpp::Publisher<coop_msgs::msg::DeviceStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr move_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace device_view

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<device_view::DeviceController>());
  rclcpp::shutdown();
  return 0;
}
