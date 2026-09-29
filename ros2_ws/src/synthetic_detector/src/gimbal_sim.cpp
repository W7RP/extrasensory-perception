// SIMULATION ONLY: the physical side of an agent's gimbal. Places the
// rendered zoom camera (a floating Gazebo model with a camera sensor and no
// collision) at the agent's true position, pointing where the gimbal reports,
// so the device's picture-in-picture shows what the zoom camera really sees.
//
//   in   /sim/ground_truth       the agent's true pose
//        <ns>/gimbal/state       where the gimbal points (world yaw, pitch)
//   out  Gazebo /world/<world>/set_pose on `model` at `rate_hz` (sim time)
//
// A camera attached with a gimbal joint would need PX4's gimbal protocol end
// to end; a kinematic model moved with set_pose is enough for the image, and
// the detector's sensor model uses the same pointing.

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <coop_msgs/msg/gimbal_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "synthetic_detector/imaging.hpp"

namespace synthetic_detector
{

class GimbalSim : public rclcpp::Node
{
public:
  GimbalSim()
  : Node("gimbal_sim")
  {
    const auto world = declare_parameter<std::string>("world", "coop_field");
    model_ = declare_parameter<std::string>("model", "zoomcam_1");
    agent_frame_ = declare_parameter<std::string>("agent_frame", "agent_1");
    const auto off = declare_parameter<std::vector<double>>("mount_offset_body", {0.0, 0.0, -0.15});
    offset_ = Vec3(off.at(0), off.at(1), off.at(2));
    service_ = "/world/" + world + "/set_pose";
    gt_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/sim/ground_truth", 10,
        [this](const tf2_msgs::msg::TFMessage & m) {
          for (const auto & t : m.transforms) {
            if (t.child_frame_id == agent_frame_) {
              Pose p;
              p.p = Vec3(t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z);
              p.q = Quat(t.transform.rotation.w, t.transform.rotation.x,
                t.transform.rotation.y, t.transform.rotation.z).normalized();
              agent_ = p;
            }
          }
        });
    state_sub_ = create_subscription<coop_msgs::msg::GimbalState>("gimbal/state", 10,
        [this](const coop_msgs::msg::GimbalState & m) {state_ = m;});
    timer_ = create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / declare_parameter<double>("rate_hz", 30.0)),
        [this]() {on_timer();});
    RCLCPP_INFO(get_logger(), "moving %s with %s's gimbal", model_.c_str(), agent_frame_.c_str());
  }

private:
  void on_timer()
  {
    if (!agent_ || !state_) {
      return;
    }
    const Pose c = gimbal_camera_pose(*agent_, offset_, state_->yaw, state_->pitch_down);
    gz::msgs::Pose req;
    req.set_name(model_);
    req.mutable_position()->set_x(c.p.x());
    req.mutable_position()->set_y(c.p.y());
    req.mutable_position()->set_z(c.p.z());
    req.mutable_orientation()->set_w(c.q.w());
    req.mutable_orientation()->set_x(c.q.x());
    req.mutable_orientation()->set_y(c.q.y());
    req.mutable_orientation()->set_z(c.q.z());
    gz::msgs::Boolean rep;
    bool result = false;
    if (!gz_.Request(service_, req, 500, rep, result) || !result || !rep.data()) {
      if (++failures_ % 30 == 1) {
        RCLCPP_WARN(get_logger(), "set_pose %s failed (%lu so far)", model_.c_str(),
          static_cast<unsigned long>(failures_));
      }
    }
  }

  std::string model_;
  std::string agent_frame_;
  std::string service_;
  Vec3 offset_{Vec3::Zero()};
  std::optional<Pose> agent_;
  std::optional<coop_msgs::msg::GimbalState> state_;
  std::uint64_t failures_{0};
  gz::transport::Node gz_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr gt_sub_;
  rclcpp::Subscription<coop_msgs::msg::GimbalState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace synthetic_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<synthetic_detector::GimbalSim>());
  rclcpp::shutdown();
  return 0;
}
