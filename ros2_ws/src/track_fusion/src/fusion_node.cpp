// Track fusion node: detections from the configured agents in, fused tracks
// out. The same node, given one agent, is the single-agent baseline the
// evaluation compares against.
//
//   in   /agent_<n>/detections   coop_msgs/DetectionArray, n in `agents`
//   out  <ns>/tracks             coop_msgs/TrackArray, all tracks predicted to now
//        <ns>/markers            visualization_msgs/MarkerArray for RViz
//        /diagnostics            real-time statistics (name = this node's full name)
//
// Threads, executors and callback groups (same design as agent_estimation's
// eskf_node, which this follows deliberately)
// ---------------------------------------------------------------------------
//   ingest thread   SingleThreadedExecutor spinning ONLY `ingest_group_`
//                   (MutuallyExclusive): one subscription per agent. It owns the
//                   reorder buffer and the Tracker, so it needs no locks. Each
//                   callback converts the message into a fixed-size batch,
//                   buffers it, and runs the tracker on every batch that is
//                   ready (see reorder_buffer.hpp).
//   output thread   SingleThreadedExecutor spinning the node's default group:
//                   the track publisher (sim-time timer), diagnostics, and
//                   parameter services. Everything that allocates (messages,
//                   markers, strings, logging) happens here.
//   hand-off        The ingest thread copies the track table into a snapshot
//                   under a mutex taken with try_lock(): it never blocks. If the
//                   output thread holds the lock at that instant, the copy is
//                   skipped (and counted); the next batch, ~50 ms later at most,
//                   writes a fresh one.
//
// Hot-path guarantees, measured live and published on /diagnostics:
//   * bounded time: the tracker's work is bounded by kMaxTracks x
//     kMaxDetections; the reorder buffer by its fixed capacity;
//   * no heap allocation in the ingest callback bodies (agent_estimation's
//     alloc_probe counts any).
// Unlike the ESKF's PX4 inputs, DetectionArray cannot come from a pre-allocated
// message pool: rclcpp's MessagePoolMemoryStrategy only accepts fixed-size
// messages, and a fixed Detection[16] array would put all 16 slots on the wire
// every frame, which the Phase 3 bandwidth budget rules out. So rclcpp's take
// path allocates the incoming message; ingest_thread_allocations counts that
// separately from hot_path_allocations (our code).
//
// Time: every stamp is simulation time. Tracks are published predicted to the
// node's current sim time, so their covariance visibly grows while nobody sees
// the entity.

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sched.h>

#include <Eigen/Eigenvalues>
#include <rclcpp/rclcpp.hpp>

#include <coop_msgs/msg/detection_array.hpp>
#include <coop_msgs/msg/track_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "agent_estimation/alloc_probe.hpp"
#include "agent_estimation/timing_stats.hpp"
#include "track_fusion/reorder_buffer.hpp"
#include "track_fusion/tracker.hpp"

using coop_msgs::msg::DetectionArray;
namespace alloc_probe = agent_estimation::alloc_probe;

namespace track_fusion
{

namespace
{
double to_seconds(const builtin_interfaces::msg::Time & t) noexcept
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

builtin_interfaces::msg::Time to_msg(double t)
{
  builtin_interfaces::msg::Time m;
  const double s = std::floor(t);
  m.sec = static_cast<std::int32_t>(s);
  m.nanosec = static_cast<std::uint32_t>(std::lround((t - s) * 1e9) % 1'000'000'000L);
  return m;
}

std::int64_t elapsed_us(std::chrono::steady_clock::time_point since) noexcept
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - since).count();
}

// What the output thread needs: the track table and counters, copied whole.
struct Snapshot
{
  bool valid{false};
  double tracker_time{0.0};
  TrackSet tracks;
  TrackerCounters counters;
};
}  // namespace

class FusionNode : public rclcpp::Node
{
public:
  FusionNode()
  : Node("track_fusion"),
    ingest_timing_(declare_parameter<std::int64_t>("ingest_callback_budget_us", 500))
  {
    TrackerConfig cfg;
    cfg.process_noise = declare_parameter<double>("process_noise", cfg.process_noise);
    cfg.gate_d2 = declare_parameter<double>("gate_d2", cfg.gate_d2);
    cfg.init_velocity_sigma =
      declare_parameter<double>("init_velocity_sigma", cfg.init_velocity_sigma);
    cfg.confirm_hits = static_cast<std::uint32_t>(
      declare_parameter<int>("confirm_hits", static_cast<int>(cfg.confirm_hits)));
    cfg.tentative_timeout_s =
      declare_parameter<double>("tentative_timeout_s", cfg.tentative_timeout_s);
    cfg.coast_after_s = declare_parameter<double>("coast_after_s", cfg.coast_after_s);
    cfg.delete_after_s = declare_parameter<double>("delete_after_s", cfg.delete_after_s);
    cfg.max_position_sigma_m =
      declare_parameter<double>("max_position_sigma_m", cfg.max_position_sigma_m);
    cfg.contribution_window_s =
      declare_parameter<double>("contribution_window_s", cfg.contribution_window_s);
    cfg_ = cfg;
    const auto assoc_name = declare_parameter<std::string>("associator", "nearest_neighbour");
    auto assoc = make_associator(assoc_name);
    if (!assoc) {
      throw std::invalid_argument("unknown associator '" + assoc_name +
              "' (nearest_neighbour, hungarian)");
    }
    tracker_ = std::make_unique<Tracker>(cfg, std::move(assoc));
    reorder_ = std::make_unique<ReorderBuffer>(declare_parameter<double>("max_hold_s", 0.3));

    const auto agents = declare_parameter<std::vector<std::int64_t>>("agents", {1, 2});
    const auto pattern = declare_parameter<std::string>("detection_topic_pattern",
        "/agent_{}/detections");
    const double publish_hz = declare_parameter<double>("publish_rate_hz", 10.0);
    producer_ = static_cast<std::uint8_t>(declare_parameter<int>("producer", 0));
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    declare_parameter<int>("ingest_thread_priority", 0);  // read by main()

    ingest_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = ingest_group_;
    // Reliable, as detectors publish: on a perfect link nothing is lost. The
    // Phase 3 link model sits between the two and decides what arrives.
    const auto qos = rclcpp::QoS(20).reliable();
    for (const auto a : agents) {
      if (a < 1 || a > static_cast<std::int64_t>(kMaxAgents)) {
        throw std::invalid_argument("agent ids must be 1.." + std::to_string(kMaxAgents));
      }
      std::string topic = pattern;
      const auto pos = topic.find("{}");
      if (pos == std::string::npos) {
        throw std::invalid_argument("detection_topic_pattern needs a {} for the agent id");
      }
      topic.replace(pos, 2, std::to_string(a));
      subs_.push_back(create_subscription<DetectionArray>(topic, qos,
          [this](const DetectionArray & m) {on_detections(m);}, opts));
      agent_list_ += (agent_list_.empty() ? "" : ",") + std::to_string(a);
    }

    tracks_pub_ = create_publisher<coop_msgs::msg::TrackArray>("tracks", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    publish_timer_ = create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / publish_hz), [this]() {publish_tracks();});
    diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_diagnostics();});

    RCLCPP_INFO(get_logger(), "fusing agents [%s] with %s association, publishing %s at %.0f Hz",
      agent_list_.c_str(), tracker_->associator_name().data(), tracks_pub_->get_topic_name(),
      publish_hz);
  }

  [[nodiscard]] rclcpp::CallbackGroup::SharedPtr ingest_group() const {return ingest_group_;}

private:
  // ------------------------------------------------------ ingest thread
  void on_detections(const DetectionArray & m) noexcept
  {
    alloc_probe::HotPathScope hot;
    const auto t0 = std::chrono::steady_clock::now();

    batch_.t = to_seconds(m.stamp);
    batch_.agent = m.source_agent;
    batch_.frame_seq = m.frame_seq;
    batch_.count = std::min(m.detections.size(), kMaxDetections);
    for (std::size_t i = 0; i < batch_.count; ++i) {
      const auto & d = m.detections[i];
      Detection & o = batch_.detections[i];
      o.z = Vec3(d.position[0], d.position[1], d.position[2]);
      const auto & c = d.covariance;  // upper triangle xx xy xz yy yz zz
      o.R << c[0], c[1], c[2],
        c[1], c[3], c[4],
        c[2], c[4], c[5];
      o.class_id = d.class_id;
      o.confidence = d.confidence;
    }
    if (m.source_agent >= 1 && m.source_agent <= kMaxAgents) {
      // Frame counter gaps: frames this node never received.
      auto & last = last_seq_[m.source_agent - 1U];
      if (last.has && m.frame_seq > last.seq + 1) {
        frames_missing_.fetch_add(m.frame_seq - last.seq - 1, std::memory_order_relaxed);
      }
      last = {true, m.frame_seq};
    }

    if (reorder_->push(batch_, ready_)) {
      run(ready_);  // buffer full: its oldest batch had to go now
    }
    while (reorder_->pop_ready(ready_)) {
      run(ready_);
    }
    write_snapshot();
    ingest_timing_.record(elapsed_us(t0));
  }

  void run(const DetectionBatch & b) noexcept
  {
    if (!tracker_->process(b)) {
      late_batches_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void write_snapshot() noexcept
  {
    std::unique_lock lock(snapshot_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      snapshot_skips_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    snapshot_.valid = true;
    snapshot_.tracker_time = tracker_->time();
    snapshot_.tracks = tracker_->tracks();
    snapshot_.counters = tracker_->counters();
    reorder_depth_max_.store(reorder_->max_depth(), std::memory_order_relaxed);
    reorder_forced_.store(reorder_->forced_releases(), std::memory_order_relaxed);
  }

  // ------------------------------------------------------ output thread
  void publish_tracks()
  {
    {
      std::lock_guard lock(snapshot_mutex_);
      out_ = snapshot_;
    }
    const double t = now().seconds();
    coop_msgs::msg::TrackArray msg;
    msg.stamp = to_msg(t);
    msg.producer = producer_;
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (std::size_t i = 0; out_.valid && i < out_.tracks.count; ++i) {
      const Track tr = predicted(out_.tracks.tracks[i], t, cfg_.process_noise);
      coop_msgs::msg::Track o;
      o.id = tr.id;
      o.class_id = tr.class_id;
      o.status = static_cast<std::uint8_t>(tr.status);
      o.source_mask = source_mask(tr, out_.tracker_time, cfg_.contribution_window_s);
      o.last_update = to_msg(tr.last_update);
      for (int k = 0; k < 3; ++k) {
        o.position[static_cast<std::size_t>(k)] = static_cast<float>(tr.x[k]);
        o.velocity[static_cast<std::size_t>(k)] = static_cast<float>(tr.x[k + 3]);
      }
      const auto & P = tr.P;
      o.covariance = {static_cast<float>(P(0, 0)), static_cast<float>(P(0, 1)),
        static_cast<float>(P(0, 2)), static_cast<float>(P(1, 1)),
        static_cast<float>(P(1, 2)), static_cast<float>(P(2, 2))};
      msg.tracks.push_back(o);
      add_markers(tr, o.source_mask, markers);
    }
    tracks_pub_->publish(msg);
    markers_pub_->publish(markers);
  }

  // Sphere at the estimate, a flat 2-sigma horizontal ellipse, and a label.
  // Colour = how many agents currently contribute detections to the track:
  // green 2+, amber 1, grey 0 (coasting on prediction alone).
  void add_markers(const Track & tr, std::uint8_t mask, visualization_msgs::msg::MarkerArray & out)
  {
    const int seeing = std::popcount(mask);
    std_msgs::msg::ColorRGBA col;
    col.a = tr.status == TrackStatus::kTentative ? 0.35F : 0.9F;
    if (seeing >= 2) {
      col.r = 0.1F; col.g = 0.75F; col.b = 0.2F;
    } else if (seeing == 1) {
      col.r = 1.0F; col.g = 0.65F; col.b = 0.0F;
    } else {
      col.r = 0.55F; col.g = 0.55F; col.b = 0.55F;
    }
    visualization_msgs::msg::Marker base;
    base.header.frame_id = frame_id_;
    base.header.stamp = now();
    base.ns = "track";
    base.action = visualization_msgs::msg::Marker::ADD;
    base.pose.position.x = tr.x[0];
    base.pose.position.y = tr.x[1];
    base.pose.position.z = tr.x[2];
    base.pose.orientation.w = 1.0;
    base.color = col;

    auto sphere = base;
    sphere.id = static_cast<std::int32_t>(tr.id * 3);
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4;
    out.markers.push_back(sphere);

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(tr.P.block<2, 2>(0, 0));
    const Eigen::Vector2d sig = es.eigenvalues().cwiseMax(0.0).cwiseSqrt();
    const Eigen::Vector2d major = es.eigenvectors().col(1);
    auto ellipse = base;
    ellipse.id = static_cast<std::int32_t>(tr.id * 3 + 1);
    ellipse.ns = "uncertainty_2sigma";
    ellipse.type = visualization_msgs::msg::Marker::CYLINDER;
    ellipse.scale.x = std::max(4.0 * sig[1], 0.05);  // 2 sigma each side
    ellipse.scale.y = std::max(4.0 * sig[0], 0.05);
    ellipse.scale.z = 0.02;
    const double yaw = std::atan2(major.y(), major.x());
    ellipse.pose.orientation.z = std::sin(0.5 * yaw);
    ellipse.pose.orientation.w = std::cos(0.5 * yaw);
    ellipse.pose.position.z = 0.02;
    ellipse.color.a = 0.35F;
    out.markers.push_back(ellipse);

    auto text = base;
    text.id = static_cast<std::int32_t>(tr.id * 3 + 2);
    text.ns = "label";
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.pose.position.z += 1.0;
    text.scale.z = 0.5;
    text.color.r = text.color.g = text.color.b = 1.0F;
    text.text = "T" + std::to_string(tr.id) + " (" + std::to_string(seeing) + " agent" +
      (seeing == 1 ? "" : "s") + ")";
    out.markers.push_back(text);
  }

  void publish_diagnostics()
  {
    Snapshot s;
    {
      std::lock_guard lock(snapshot_mutex_);
      s = snapshot_;
    }
    const auto cb = ingest_timing_.snapshot();
    const auto hot = alloc_probe::hot_path_allocations();
    const auto thread_allocs = alloc_probe::tracked_thread_allocations();
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = std::string(get_fully_qualified_name()) + ": fusion";
    st.hardware_id = "track_fusion";
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "running";
    if (hot > 0) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "heap allocation detected in hot path";
    } else if (cb.overruns > 0) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "ingest callback exceeded its time budget";
    }
    auto kv = [&st](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = v;
        st.values.push_back(e);
      };
    auto n = [](auto v) {return std::to_string(v);};
    const auto & c = s.counters;
    kv("agents", agent_list_);
    kv("associator", std::string(tracker_->associator_name()));
    kv("detection_callbacks", n(cb.count));
    kv("detection_cb_mean_us", n(cb.mean_us));
    kv("detection_cb_p99_us", n(cb.p99_us));
    kv("detection_cb_max_us", n(cb.max_us));
    kv("detection_cb_budget_us", n(cb.budget_us));
    kv("detection_cb_overruns", n(cb.overruns));
    kv("hot_path_allocations", n(hot));
    kv("ingest_thread_allocations", n(thread_allocs));
    kv("snapshot_skips", n(snapshot_skips_.load()));
    kv("batches", n(c.batches));
    kv("detections", n(c.detections));
    kv("associated", n(c.associated));
    kv("births", n(c.births));
    kv("births_refused", n(c.births_refused));
    kv("confirmations", n(c.confirmations));
    kv("deletions", n(c.deletions));
    kv("live_tracks", n(s.tracks.count));
    kv("late_batches", n(late_batches_.load()));
    kv("frames_missing", n(frames_missing_.load()));
    kv("reorder_depth_max", n(reorder_depth_max_.load()));
    kv("reorder_forced_releases", n(reorder_forced_.load()));
    arr.status.push_back(st);
    diag_pub_->publish(arr);

    if (++diag_ticks_ % 10 == 0) {
      RCLCPP_INFO(get_logger(),
        "%s | %zu tracks | detection cb mean %.0f us p99 %ld us max %ld us, overruns %lu | "
        "hot-path allocs %lu | births %lu, deletions %lu, late %lu",
        st.message.c_str(), s.tracks.count, cb.mean_us, static_cast<long>(cb.p99_us),
        static_cast<long>(cb.max_us), static_cast<unsigned long>(cb.overruns),
        static_cast<unsigned long>(hot), static_cast<unsigned long>(c.births),
        static_cast<unsigned long>(c.deletions),
        static_cast<unsigned long>(late_batches_.load()));
    }
  }

  // ---------------------------------------------------------------- state
  TrackerConfig cfg_;          // immutable after construction: read by both threads
  std::uint8_t producer_{0};
  std::string frame_id_;
  std::string agent_list_;

  // Ingest-thread-owned.
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<ReorderBuffer> reorder_;
  DetectionBatch batch_;
  DetectionBatch ready_;
  struct SeqState
  {
    bool has{false};
    std::uint32_t seq{0};
  };
  std::array<SeqState, kMaxAgents> last_seq_{};

  // Shared (atomics, or the try_lock-guarded snapshot).
  agent_estimation::TimingStats ingest_timing_;
  std::atomic<std::uint64_t> snapshot_skips_{0};
  std::atomic<std::uint64_t> late_batches_{0};
  std::atomic<std::uint64_t> frames_missing_{0};
  std::atomic<std::size_t> reorder_depth_max_{0};
  std::atomic<std::uint64_t> reorder_forced_{0};
  std::mutex snapshot_mutex_;
  Snapshot snapshot_;

  // Output-thread-owned.
  Snapshot out_;
  std::uint64_t diag_ticks_{0};

  rclcpp::CallbackGroup::SharedPtr ingest_group_;
  std::vector<rclcpp::Subscription<DetectionArray>::SharedPtr> subs_;
  rclcpp::Publisher<coop_msgs::msg::TrackArray>::SharedPtr tracks_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace track_fusion

namespace
{
// Optional SCHED_FIFO for the ingest thread (as in eskf_node): needs rtprio,
// which WSL2 does not grant; a companion computer can.
void set_realtime_priority(int priority, const rclcpp::Logger & logger)
{
  if (priority <= 0) {
    return;
  }
  sched_param sp{};
  sp.sched_priority = priority;
  const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (rc == 0) {
    RCLCPP_INFO(logger, "ingest thread: SCHED_FIFO priority %d", priority);
  } else {
    RCLCPP_WARN(logger, "ingest thread: SCHED_FIFO %d refused (%s)", priority, std::strerror(rc));
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<track_fusion::FusionNode>();
  const int priority = static_cast<int>(node->get_parameter("ingest_thread_priority").as_int());

  rclcpp::executors::SingleThreadedExecutor ingest_exec;
  ingest_exec.add_callback_group(node->ingest_group(), node->get_node_base_interface());
  rclcpp::executors::SingleThreadedExecutor output_exec;
  output_exec.add_node(node);

  std::thread ingest_thread([&]() {
      alloc_probe::track_this_thread();
      set_realtime_priority(priority, node->get_logger());
      ingest_exec.spin();
    });
  output_exec.spin();

  ingest_exec.cancel();
  ingest_thread.join();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
