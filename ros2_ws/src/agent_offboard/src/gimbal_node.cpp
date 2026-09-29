// Gimbal controller for one agent's zoom camera: points it at whatever the
// planner asks, as fast as the gimbal slews, and reports where it points.
//
//   in   <ns>/gimbal/command   coop_msgs/GimbalCommand (a world point, usually
//                              a track); with none for `command_timeout_s`,
//                              or `active` false, it stows (straight down)
//        PX4 vehicle_local_position: the agent's own position (NED from its
//                              start point, origin_world_enu)
//   out  <ns>/gimbal/state     coop_msgs/GimbalState at `rate_hz`
//
// Runs on the agent, like the offboard node. In simulation the camera's
// pointing is applied to the rendered zoom camera by synthetic_detector's
// gimbal_sim; on hardware it would go to the gimbal over its own protocol
// (e.g. MAVLink gimbal v2) and the state would come back from it.

#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <coop_msgs/msg/gimbal_command.hpp>
#include <coop_msgs/msg/gimbal_state.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "agent_offboard/gimbal.hpp"
#include "agent_offboard/px4_topics.hpp"

namespace agent_offboard
{

class GimbalNode : public rclcpp::Node
{
public:
  GimbalNode()
  : Node("gimbal")
  {
    agent_id_ = static_cast<std::uint8_t>(declare_parameter<int>("agent_id", 1));
    const auto ns = declare_parameter<std::string>("px4_namespace", "");
    const auto origin = declare_parameter<std::vector<double>>("origin_world_enu", {0.0, 0.0, 0.0});
    const auto offset = declare_parameter<std::vector<double>>("mount_offset_enu", {0.0, 0.0, -0.15});
    if (origin.size() != 3 || offset.size() != 3) {
      throw std::invalid_argument("origin_world_enu and mount_offset_enu need 3 values");
    }
    origin_ = {origin[0], origin[1], origin[2]};
    offset_ = {offset[0], offset[1], offset[2]};
    const double rate_hz = declare_parameter<double>("rate_hz", 30.0);
    dt_ = 1.0 / rate_hz;
    cfg_.slew_rate_rad_s = declare_parameter<double>("slew_rate_dps", 90.0) * M_PI / 180.0;
    cfg_.hfov_rad = declare_parameter<double>("hfov_deg", 6.0) * M_PI / 180.0;
    vfov_rad_ = declare_parameter<double>("vfov_deg", 3.375) * M_PI / 180.0;
    cfg_.min_pitch_down_rad = declare_parameter<double>("min_pitch_down_deg", 0.0) * M_PI / 180.0;
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 2.0);

    pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      px4_topic<px4_msgs::msg::VehicleLocalPosition>(ns, "/fmu/out/vehicle_local_position"),
      rclcpp::SensorDataQoS(), [this](const px4_msgs::msg::VehicleLocalPosition & m) {
        if (m.xy_valid && m.z_valid) {
          // NED from the start point -> world ENU.
          position_ = std::array<double, 3>{origin_[0] + m.y + offset_[0],
            origin_[1] + m.x + offset_[1], origin_[2] - m.z + offset_[2]};
        }
      });
    cmd_sub_ = create_subscription<coop_msgs::msg::GimbalCommand>("gimbal/command", 10,
        [this](const coop_msgs::msg::GimbalCommand & m) {
          command_ = m;
          command_time_ = now();
        });
    state_pub_ = create_publisher<coop_msgs::msg::GimbalState>("gimbal/state", 10);
    timer_ = create_timer(this, get_clock(), rclcpp::Duration::from_seconds(dt_),
        [this]() {on_timer();});
    RCLCPP_INFO(get_logger(), "agent %u gimbal: %.1f x %.1f deg field, %.0f deg/s, %.0f Hz",
      agent_id_, cfg_.hfov_rad * 180 / M_PI, vfov_rad_ * 180 / M_PI,
      cfg_.slew_rate_rad_s * 180 / M_PI, rate_hz);
  }

private:
  void on_timer()
  {
    GimbalAngles cmd{current_.yaw, M_PI / 2};  // stowed: straight down
    std::uint32_t track = 0;
    const bool fresh = command_ &&
      (now() - command_time_).seconds() < command_timeout_s_;
    if (fresh && command_->active && position_) {
      cmd = pointing_to(*position_,
          {command_->target[0], command_->target[1], command_->target[2]}, cfg_);
      track = command_->track_id;
    }
    current_ = step(current_, cmd, dt_, cfg_);

    coop_msgs::msg::GimbalState s;
    s.stamp = now();
    s.agent = agent_id_;
    s.yaw = static_cast<float>(current_.yaw);
    s.pitch_down = static_cast<float>(current_.pitch_down);
    s.hfov = static_cast<float>(cfg_.hfov_rad);
    s.vfov = static_cast<float>(vfov_rad_);
    s.track_id = track;
    s.on_target = track != 0 && on_target(current_, cmd, cfg_);
    state_pub_->publish(s);
  }

  std::uint8_t agent_id_{1};
  std::array<double, 3> origin_{};
  std::array<double, 3> offset_{};
  double dt_{1.0 / 30.0};
  GimbalConfig cfg_;
  double vfov_rad_{0.0589};
  double command_timeout_s_{2.0};

  std::optional<std::array<double, 3>> position_;
  std::optional<coop_msgs::msg::GimbalCommand> command_;
  rclcpp::Time command_time_{0, 0, RCL_ROS_TIME};
  GimbalAngles current_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<coop_msgs::msg::GimbalCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<coop_msgs::msg::GimbalState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agent_offboard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agent_offboard::GimbalNode>());
  rclcpp::shutdown();
  return 0;
}
