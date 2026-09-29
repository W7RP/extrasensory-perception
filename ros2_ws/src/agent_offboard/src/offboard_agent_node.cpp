// Offboard node for one agent: take off, fly with velocity setpoints, land.
// Where it flies (`route_source`):
//   params  a scripted waypoint list (route_nea, laps) relative to the agent's
//           start point, or a square when none is given; lands at the end;
//   goal    wherever the overwatch planner says: coop_msgs/AgentGoal on
//           ~/../goal (world frame position and camera heading), re-issued
//           every planning cycle; holds position between goals, lands when a
//           goal says `land`.
// Copied from the author's quad-autonomy-sim (quad_offboard); the goal mode
// replaces its planner-path mode.
//
// Talks to PX4 only through the uXRCE-DDS /fmu/{in,out} topics, so the same node
// runs unchanged against SITL or a real Pixhawk connected to a companion
// computer over serial/Ethernet (only the agent's transport changes).
//
// Executor / callback-group design:
//   Everything (two subscriptions + the control timer) sits in ONE
//   MutuallyExclusive callback group on a SingleThreadedExecutor. Callbacks
//   therefore never run concurrently, so the latest-sample members below need no
//   locks and the timer always sees a consistent snapshot. Subscription callbacks
//   only copy a few scalars (bounded time, no allocation). This is plenty for a
//   20 Hz outer loop.
//
// Time: phase timeouts use the node clock, i.e. simulation time when
// use_sim_time is set (every node in this project runs on /clock). The control
// timer itself is a wall timer: PX4's offboard-loss check needs a steady
// setpoint stream even while the simulation clock is not advancing yet.
//
// Safety behaviour:
//   The node only commands the vehicle while PX4 reports nav_state == OFFBOARD.
//   If PX4 leaves offboard (RC takeover, failsafe, operator mode switch) the node
//   aborts and stops publishing setpoints; it never tries to re-engage by itself.

#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <coop_msgs/msg/agent_goal.hpp>

#include "agent_offboard/px4_topics.hpp"
#include "agent_offboard/waypoint_follower.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace agent_offboard
{

enum class Phase : std::uint8_t
{
  kWaitForPx4,   // no valid local position / status yet
  kPrime,        // stream setpoints so PX4 accepts the offboard switch
  kEngage,       // request OFFBOARD + ARM until both are confirmed
  kMission,      // follow the route
  kLand,         // NAV_LAND sent, waiting for auto-disarm
  kDone,
  kAborted,
};

constexpr std::string_view to_string(Phase p)
{
  switch (p) {
    case Phase::kWaitForPx4: return "WAIT_FOR_PX4";
    case Phase::kPrime: return "PRIME";
    case Phase::kEngage: return "ENGAGE";
    case Phase::kMission: return "MISSION";
    case Phase::kLand: return "LAND";
    case Phase::kDone: return "DONE";
    case Phase::kAborted: return "ABORTED";
  }
  return "?";
}

class OffboardAgentNode : public rclcpp::Node
{
public:
  OffboardAgentNode()
  : Node("offboard"),
    follower_(load_follower_config())
  {
    const auto ns = declare_parameter<std::string>("px4_namespace", "");
    side_m_ = declare_parameter<double>("square_side_m", 5.0);
    altitude_m_ = declare_parameter<double>("altitude_m", 3.0);
    const double rate_hz = declare_parameter<double>("control_rate_hz", 20.0);
    // Optional route: flat [north, east, altitude, ...] triples in metres,
    // relative to the start position (altitude above it). Empty = the square.
    route_nea_ = declare_parameter<std::vector<double>>("route_nea", std::vector<double>{});
    laps_ = static_cast<int>(declare_parameter<int>("laps", 1));
    const auto yaw_mode = declare_parameter<std::string>("yaw_mode", "hold");
    if (route_nea_.size() % 3 != 0 || laps_ < 1) {
      throw std::invalid_argument("route_nea must hold north/east/altitude triples and laps >= 1");
    }
    if (yaw_mode != "hold" && yaw_mode != "travel") {
      throw std::invalid_argument("yaw_mode must be 'hold' or 'travel'");
    }
    face_travel_ = yaw_mode == "travel";
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate_dps", 0.0) * M_PI / 180.0;
    px4_timeout_s_ = declare_parameter<double>("px4_timeout_s", 120.0);
    engage_timeout_s_ = declare_parameter<double>("engage_timeout_s", 20.0);
    shutdown_when_done_ = declare_parameter<bool>("shutdown_when_done", true);
    const auto route_source = declare_parameter<std::string>("route_source", "params");
    if (route_source != "params" && route_source != "goal") {
      throw std::invalid_argument("route_source must be 'params' or 'goal'");
    }
    goal_mode_ = route_source == "goal";
    const auto origin = declare_parameter<std::vector<double>>("origin_world_enu", {0.0, 0.0, 0.0});
    if (origin.size() != 3) {
      throw std::invalid_argument("origin_world_enu must have 3 values");
    }
    origin_enu_ = {origin[0], origin[1], origin[2]};

    if (rate_hz < 5.0) {
      // PX4 drops out of offboard if setpoints arrive slower than 2 Hz
      // (COM_OF_LOSS_T); keep a wide margin.
      throw std::invalid_argument("control_rate_hz must be >= 5");
    }

    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = group_;

    // PX4 publishes best-effort; a reliable subscriber would never match.
    const auto px4_qos = rclcpp::SensorDataQoS();

    status_sub_ = create_subscription<VehicleStatus>(
      px4_topic<VehicleStatus>(ns, "/fmu/out/vehicle_status"), px4_qos,
      [this](const VehicleStatus & msg) {
        nav_state_ = msg.nav_state;
        arming_state_ = msg.arming_state;
        target_system_ = msg.system_id;
        have_status_ = true;
      }, sub_opts);

    local_pos_sub_ = create_subscription<VehicleLocalPosition>(
      px4_topic<VehicleLocalPosition>(ns, "/fmu/out/vehicle_local_position"), px4_qos,
      [this](const VehicleLocalPosition & msg) {
        pos_ = {msg.x, msg.y, msg.z};
        vel_ = {msg.vx, msg.vy, msg.vz};
        heading_ = msg.heading;
        pos_valid_ = msg.xy_valid && msg.z_valid && msg.v_xy_valid && msg.v_z_valid;
      }, sub_opts);


    if (goal_mode_) {
      goal_sub_ = create_subscription<coop_msgs::msg::AgentGoal>("goal", 10,
          [this](const coop_msgs::msg::AgentGoal & g) {
            goal_ = g;
            new_goal_ = true;
          }, sub_opts);
    }

    offboard_mode_pub_ = create_publisher<OffboardControlMode>(
      px4_topic<OffboardControlMode>(ns, "/fmu/in/offboard_control_mode"), 10);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>(
      px4_topic<TrajectorySetpoint>(ns, "/fmu/in/trajectory_setpoint"), 10);
    command_pub_ = create_publisher<VehicleCommand>(
      px4_topic<VehicleCommand>(ns, "/fmu/in/vehicle_command"), 10);

    control_dt_ = 1.0 / rate_hz;
    const auto period = std::chrono::duration<double>(control_dt_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {on_timer();}, group_);

    RCLCPP_INFO(
      get_logger(), "%s, %.0f Hz; listening on %s",
      route_nea_.empty() ? "square route" : "custom route", rate_hz,
      status_sub_->get_topic_name());
  }

  [[nodiscard]] Phase phase() const {return phase_;}

private:
  FollowerConfig load_follower_config()
  {
    FollowerConfig c;
    c.cruise_speed_mps = declare_parameter<double>("cruise_speed_mps", c.cruise_speed_mps);
    c.vertical_speed_mps = declare_parameter<double>("vertical_speed_mps", c.vertical_speed_mps);
    c.position_gain = declare_parameter<double>("position_gain", c.position_gain);
    c.acceptance_radius_m = declare_parameter<double>("acceptance_radius_m", c.acceptance_radius_m);
    c.acceptance_speed_mps =
      declare_parameter<double>("acceptance_speed_mps", c.acceptance_speed_mps);
    return c;
  }

  // Build the route once, before the vehicle moves (allocation here is fine:
  // it is not the control loop). Waypoints are NED, anchored at the start.
  bool load_route()
  {
    std::vector<Vec3> wps;
    if (goal_mode_) {
      // Climb in place; the planner's goals take over from there.
      wps.push_back({pos_.x, pos_.y, pos_.z - altitude_m_});
    } else if (route_nea_.empty()) {
      const auto square = make_square(pos_, side_m_, altitude_m_);
      wps.assign(square.begin(), square.end());
    } else {
      wps.push_back({pos_.x, pos_.y, pos_.z - route_nea_[2]});  // climb in place first
      for (int lap = 0; lap < laps_; ++lap) {
        for (std::size_t i = 0; i < route_nea_.size(); i += 3) {
          wps.push_back({pos_.x + route_nea_[i], pos_.y + route_nea_[i + 1],
              pos_.z - route_nea_[i + 2]});
        }
      }
    }
    std::vector<double> yaws(wps.size(), heading_);
    if (face_travel_) {
      travel_yaws(wps, heading_, yaws);
    }
    if (!follower_.set_route(wps, yaws)) {
      RCLCPP_ERROR(get_logger(), "route rejected: %zu waypoints (max %zu)", wps.size(),
        WaypointFollower::kMaxWaypoints);
      return false;
    }
    RCLCPP_INFO(get_logger(), "route: %zu waypoints, yaw %s", wps.size(),
      face_travel_ ? "facing travel" : "held");
    return true;
  }

  void set_phase(Phase next)
  {
    if (next != phase_) {
      RCLCPP_INFO(get_logger(), "%s -> %s", to_string(phase_).data(), to_string(next).data());
      phase_ = next;
      phase_start_ = now();
    }
  }

  [[nodiscard]] double seconds_in_phase() {return (now() - phase_start_).seconds();}
  [[nodiscard]] bool in_offboard() const
  {
    return nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }
  [[nodiscard]] bool armed() const {return arming_state_ == VehicleStatus::ARMING_STATE_ARMED;}

  void on_timer()
  {
    if (!clock_started_) {
      // With use_sim_time the clock reads 0 until the first /clock message.
      // Start the first phase's timeout only once time is real.
      if (now().nanoseconds() == 0) {
        return;
      }
      clock_started_ = true;
      phase_start_ = now();
    }
    switch (phase_) {
      case Phase::kWaitForPx4:
        if (have_status_ && pos_valid_) {
          // Route is anchored at wherever the vehicle is sitting when PX4 is ready.
          if (!load_route()) {
            set_phase(Phase::kAborted);
            return;
          }
          hold_yaw_ = heading_;
          yaw_setpoint_ = heading_;
          set_phase(Phase::kPrime);
        } else if (seconds_in_phase() > px4_timeout_s_) {
          RCLCPP_ERROR(get_logger(), "no valid PX4 status/local position within %.0f s "
            "(status %s, position %s)", px4_timeout_s_, have_status_ ? "ok" : "missing",
            pos_valid_ ? "valid" : "invalid");
          set_phase(Phase::kAborted);
        }
        return;

      case Phase::kPrime:
        publish_velocity({0.0, 0.0, 0.0}, hold_yaw_);
        // PX4 requires a setpoint stream before it accepts the mode switch; 1 s is ample.
        if (seconds_in_phase() > 1.0) {
          set_phase(Phase::kEngage);
        }
        return;

      case Phase::kEngage:
        publish_velocity({0.0, 0.0, 0.0}, hold_yaw_);
        if (in_offboard() && armed()) {
          set_phase(Phase::kMission);
          return;
        }
        if ((now() - last_command_).seconds() > 1.0) {
          // Re-sent once a second: arming is rejected until the EKF and preflight
          // checks are happy, which can take a few seconds after SITL starts.
          send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);  // custom, OFFBOARD
          send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
          last_command_ = now();
        }
        if (seconds_in_phase() > engage_timeout_s_) {
          RCLCPP_ERROR(get_logger(), "PX4 did not enter OFFBOARD+ARMED within %.0f s",
            engage_timeout_s_);
          set_phase(Phase::kAborted);
        }
        return;

      case Phase::kMission: {
          if (!in_offboard()) {
            RCLCPP_WARN(get_logger(), "PX4 left OFFBOARD (nav_state=%u); handing control back",
              nav_state_);
            set_phase(Phase::kAborted);
            return;
          }
          if (goal_mode_ && goal_ && goal_->land) {
            RCLCPP_INFO(get_logger(), "planner: land");
            send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            set_phase(Phase::kLand);
            return;
          }
          if (goal_mode_ && new_goal_ && goal_ && climbed_) {
            apply_goal(*goal_);
          }
          const std::size_t before = follower_.active_index();
          const auto cmd = follower_.step(pos_, vel_);
          if (follower_.active_index() != before && !follower_.finished()) {
            RCLCPP_INFO(get_logger(), "waypoint %zu/%zu reached", before + 1, follower_.size());
          }
          if (!cmd && goal_mode_) {
            // At the goal (or still climbing done, no goal yet): hold, facing
            // the goal's heading, and keep streaming setpoints.
            climbed_ = true;
            yaw_setpoint_ = step_yaw(yaw_setpoint_, hold_yaw_, max_yaw_rate_ * control_dt_);
            publish_velocity({0.0, 0.0, 0.0}, yaw_setpoint_);
            return;
          }
          if (!cmd) {
            RCLCPP_INFO(get_logger(), "route complete, landing");
            send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            set_phase(Phase::kLand);
            return;
          }
          // Rate-limit the yaw setpoint (max_yaw_rate_dps; 0 = unlimited).
          yaw_setpoint_ = step_yaw(yaw_setpoint_, cmd->yaw_rad, max_yaw_rate_ * control_dt_);
          publish_velocity(cmd->velocity_ned, yaw_setpoint_);
          return;
        }

      case Phase::kLand:
        // PX4 auto-disarms after touchdown (COM_DISARM_LAND).
        if (!armed()) {
          RCLCPP_INFO(get_logger(), "landed and disarmed");
          set_phase(Phase::kDone);
        }
        return;

      case Phase::kDone:
      case Phase::kAborted:
        if (shutdown_when_done_) {
          timer_->cancel();
          rclcpp::shutdown();
        }
        return;
    }
  }

  // A planner goal (world ENU) as a one-waypoint route in PX4's local NED,
  // whose origin is this agent's start point (origin_world_enu).
  void apply_goal(const coop_msgs::msg::AgentGoal & g)
  {
    new_goal_ = false;
    const Vec3 ned{g.position[1] - origin_enu_[1], g.position[0] - origin_enu_[0],
      -(g.position[2] - origin_enu_[2])};
    const double heading = std::remainder(M_PI / 2 - g.yaw, 2.0 * M_PI);  // ENU yaw -> NED heading
    const std::array<Vec3, 1> wp{ned};
    const std::array<double, 1> yaw{heading};
    follower_.set_route(wp, yaw);
    hold_yaw_ = heading;
  }

  // Timestamp for messages sent TO PX4: 0 = "stamp on arrival". The uXRCE-DDS
  // client converts a non-zero stamp with its timesync offset
  // (min(stamp - offset, now)), and PX4 treats offboard setpoints older than
  // COM_OF_LOSS_T (1 s) as lost. In SITL the simulation runs a few percent off
  // real time, so between timesync corrections that offset goes stale: fresh
  // setpoints arrived looking ~1 s old, and PX4 intermittently dropped to Hold
  // mid-mission. With 0, freshness means arrival time, as it should.
  static constexpr std::uint64_t kStampOnArrival = 0;

  void publish_velocity(const Vec3 & v_ned, double yaw)
  {
    const auto stamp_us = kStampOnArrival;

    OffboardControlMode mode{};
    mode.timestamp = stamp_us;
    mode.velocity = true;
    offboard_mode_pub_->publish(mode);

    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    TrajectorySetpoint sp{};
    sp.timestamp = stamp_us;
    sp.position = {kNaN, kNaN, kNaN};  // NaN = "not controlled" to PX4
    sp.velocity = {static_cast<float>(v_ned.x), static_cast<float>(v_ned.y),
      static_cast<float>(v_ned.z)};
    sp.acceleration = {kNaN, kNaN, kNaN};
    sp.jerk = {kNaN, kNaN, kNaN};
    sp.yaw = static_cast<float>(yaw);
    sp.yawspeed = kNaN;
    setpoint_pub_->publish(sp);
  }

  void send_command(std::uint32_t command, float p1 = 0.0F, float p2 = 0.0F)
  {
    VehicleCommand cmd{};
    cmd.timestamp = kStampOnArrival;
    cmd.command = command;
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    command_pub_->publish(cmd);
  }

  // Configuration
  double side_m_{};
  double altitude_m_{};
  std::vector<double> route_nea_;
  int laps_{1};
  bool face_travel_{false};
  double max_yaw_rate_{0.0};   // [rad/s], 0 = unlimited
  double control_dt_{0.05};
  double yaw_setpoint_{0.0};
  double px4_timeout_s_{};
  double engage_timeout_s_{};
  bool shutdown_when_done_{};
  bool clock_started_{false};
  bool goal_mode_{false};
  bool new_goal_{false};
  bool climbed_{false};
  std::array<double, 3> origin_enu_{};
  std::optional<coop_msgs::msg::AgentGoal> goal_;
  rclcpp::Subscription<coop_msgs::msg::AgentGoal>::SharedPtr goal_sub_;

  // Latest PX4 state (written by subscriptions, read by the timer; same
  // mutually-exclusive group, so no locking).
  bool have_status_{false};
  bool pos_valid_{false};
  std::uint8_t nav_state_{0};
  std::uint8_t arming_state_{0};
  std::uint8_t target_system_{1};
  Vec3 pos_{};
  Vec3 vel_{};
  double heading_{0.0};

  // Mission state
  WaypointFollower follower_;
  double hold_yaw_{0.0};
  Phase phase_{Phase::kWaitForPx4};
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agent_offboard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agent_offboard::OffboardAgentNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  const bool ok = node->phase() == agent_offboard::Phase::kDone;
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return ok ? 0 : 1;
}
