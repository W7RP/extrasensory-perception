// Overwatch planner node: re-tasks every agent once per cycle so that, between
// them, they see what the ground device cannot (planner.hpp has the method).
//
//   in   /device/status        the device's pose (sent up from the device)
//        /device/tracks        tracks it holds: keep watching those people
//        /agent_<n>/status     each agent's own position report
//   out  /agent_<n>/goal       coop_msgs/AgentGoal, every cycle
//
// Everything it uses is what the device and agents report or hold: no ground
// truth. The map (occluders) is the world file, which the device is assumed to
// have. The session ends with `land` goals after `duration_s` of planning
// (0 = never), or when the game view quits (/game/quit).

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <coop_msgs/msg/agent_goal.hpp>
#include <coop_msgs/msg/agent_status.hpp>
#include <coop_msgs/msg/device_status.hpp>
#include <coop_msgs/msg/track_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include "overwatch/planner.hpp"
#include "synthetic_detector/sdf_occluders.hpp"

namespace overwatch
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("overwatch_planner")
  {
    PlannerConfig cfg;
    cfg.radius_m = declare_parameter<double>("radius_m", cfg.radius_m);
    cfg.interest_radius_m = declare_parameter<double>("interest_radius_m", cfg.interest_radius_m);
    cfg.cell_m = declare_parameter<double>("cell_m", cfg.cell_m);
    cfg.candidates = static_cast<int>(declare_parameter<int>("candidates", cfg.candidates));
    cfg.min_separation_m = declare_parameter<double>("min_separation_m", cfg.min_separation_m);
    cfg.hysteresis = declare_parameter<double>("hysteresis", cfg.hysteresis);
    const auto b = declare_parameter<std::vector<double>>("bounds_xy", {-1e9, 1e9, -1e9, 1e9});
    cfg.xmin = b.at(0);
    cfg.xmax = b.at(1);
    cfg.ymin = b.at(2);
    cfg.ymax = b.at(3);
    const double d2r = M_PI / 180.0;
    cfg.camera.hfov_rad = declare_parameter<double>("camera.hfov_deg", 90.0) * d2r;
    cfg.camera.vfov_rad = declare_parameter<double>("camera.vfov_deg", 70.0) * d2r;
    cfg.camera.pitch_down_rad = declare_parameter<double>("camera.pitch_down_deg", 40.0) * d2r;
    cfg.camera.max_range_m = declare_parameter<double>("camera.max_range_m", 35.0);
    cfg.camera.min_range_m = declare_parameter<double>("camera.min_range_m", 1.0);
    cfg.camera.mount_offset_body = Vec3::Zero();
    agents_ = declare_parameter<std::vector<std::int64_t>>("agents", {1, 2});
    altitudes_ = declare_parameter<std::vector<double>>("altitudes_m", {12.0, 14.0});
    if (altitudes_.size() != agents_.size()) {
      throw std::invalid_argument("altitudes_m needs one altitude per agent");
    }
    duration_s_ = declare_parameter<double>("duration_s", 0.0);
    const double rate_hz = declare_parameter<double>("rate_hz", 1.0);
    planner_ = std::make_unique<Planner>(cfg, synthetic_detector::load_occluders(
          declare_parameter<std::string>("world_file", ""),
          declare_parameter<std::vector<std::string>>("model_paths", std::vector<std::string>{}),
          declare_parameter<std::vector<std::string>>("exclude_models", {"entity"})));

    device_sub_ = create_subscription<coop_msgs::msg::DeviceStatus>("/device/status", 5,
        [this](const coop_msgs::msg::DeviceStatus & m) {device_ = m;});
    tracks_sub_ = create_subscription<coop_msgs::msg::TrackArray>("/device/tracks", 5,
        [this](coop_msgs::msg::TrackArray::ConstSharedPtr m) {tracks_ = m;});
    quit_sub_ = create_subscription<std_msgs::msg::Bool>("/game/quit",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool & m) {land_ = land_ || m.data;});
    for (const auto a : agents_) {
      status_subs_.push_back(create_subscription<coop_msgs::msg::AgentStatus>(
          "/agent_" + std::to_string(a) + "/status", 5,
          [this](const coop_msgs::msg::AgentStatus & m) {
            positions_[m.agent] = Vec3(m.position[0], m.position[1], m.position[2]);
          }));
      goal_pubs_.push_back(create_publisher<coop_msgs::msg::AgentGoal>(
          "/agent_" + std::to_string(a) + "/goal", 10));
    }
    current_.resize(agents_.size());
    timer_ = create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz),
        [this]() {cycle();});
    RCLCPP_INFO(get_logger(), "overwatch: %zu agents, ring %.0f m around the device, %.0f s",
      agents_.size(), cfg.radius_m, duration_s_);
  }

private:
  void cycle()
  {
    if (!device_) {
      return;
    }
    const rclcpp::Time t = now();
    if (started_.nanoseconds() == 0) {
      started_ = t;
    }
    if (duration_s_ > 0.0 && (t - started_).seconds() > duration_s_) {
      land_ = true;
    }
    if (land_) {
      for (std::size_t k = 0; k < agents_.size(); ++k) {
        coop_msgs::msg::AgentGoal g;
        g.stamp = t;
        g.agent = static_cast<std::uint8_t>(agents_[k]);
        g.land = true;
        goal_pubs_[k]->publish(g);
      }
      return;
    }
    std::vector<Vec2> tracks;
    if (tracks_) {
      for (const auto & tr : tracks_->tracks) {
        if (tr.status != coop_msgs::msg::Track::STATUS_TENTATIVE) {
          tracks.emplace_back(tr.position[0], tr.position[1]);
        }
      }
    }
    std::vector<Vec3> positions;
    for (const auto a : agents_) {
      const auto it = positions_.find(static_cast<std::uint8_t>(a));
      positions.push_back(it != positions_.end() ? it->second : Vec3::Zero());
    }
    const auto w0 = std::chrono::steady_clock::now();
    const Vec2 dev(device_->position[0], device_->position[1]);
    const auto spots = planner_->plan(dev, tracks, altitudes_, positions, current_);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count();
    for (std::size_t k = 0; k < spots.size(); ++k) {
      current_[k] = spots[k];
      coop_msgs::msg::AgentGoal g;
      g.stamp = t;
      g.agent = static_cast<std::uint8_t>(agents_[k]);
      g.position = {static_cast<float>(spots[k].position.x()),
        static_cast<float>(spots[k].position.y()), static_cast<float>(spots[k].position.z())};
      g.yaw = static_cast<float>(spots[k].yaw);
      g.score = static_cast<float>(spots[k].score);
      goal_pubs_[k]->publish(g);
    }
    if (++cycles_ % 10 == 1) {
      RCLCPP_INFO(get_logger(), "plan %.1f ms: %s", ms, [&]() {
          std::string s;
          for (std::size_t k = 0; k < spots.size(); ++k) {
            char b[96];
            std::snprintf(b, sizeof(b), "agent %ld -> (%.1f, %.1f) covers %.0f m2  ", agents_[k],
              spots[k].position.x(), spots[k].position.y(), spots[k].score);
            s += b;
          }
          return s;
        }().c_str());
    }
  }

  std::vector<std::int64_t> agents_;
  std::vector<double> altitudes_;
  double duration_s_{0.0};
  std::unique_ptr<Planner> planner_;
  std::optional<coop_msgs::msg::DeviceStatus> device_;
  coop_msgs::msg::TrackArray::ConstSharedPtr tracks_;
  std::map<std::uint8_t, Vec3> positions_;
  std::vector<std::optional<Spot>> current_;
  bool land_{false};
  rclcpp::Time started_{0, 0, RCL_ROS_TIME};
  std::uint64_t cycles_{0};

  rclcpp::Subscription<coop_msgs::msg::DeviceStatus>::SharedPtr device_sub_;
  rclcpp::Subscription<coop_msgs::msg::TrackArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr quit_sub_;
  std::vector<rclcpp::Subscription<coop_msgs::msg::AgentStatus>::SharedPtr> status_subs_;
  std::vector<rclcpp::Publisher<coop_msgs::msg::AgentGoal>::SharedPtr> goal_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace overwatch

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<overwatch::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
