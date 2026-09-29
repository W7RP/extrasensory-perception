// The device's see-through view: every track the device holds, drawn on its
// own camera image where the entity really is, including through walls.
//
//   in   <image_topic>, <info_topic>   the device camera (Gazebo, bridged)
//        <tracks_topic>                what the device fused from the agents
//        /sim/ground_truth             the device's own pose (see below), and,
//                                      for evaluation only, the true entity
//   out  ~/image                       the annotated image
//        ~/truth                       coop_msgs/OverlayTruth per entity per frame
//                                      (evaluation only)
//        optional MP4 (`video_path`) and PNG stills every `snapshot_every_s`
//        of simulation time (`snapshot_dir`)
//
// Per frame, each confirmed or coasting track is predicted to the frame's
// capture time (constant velocity) and drawn as:
//   * a 3D box of the entity's size, standing where the track says, facing its
//     direction of travel;
//   * a person silhouette (a billboard cut-out facing the camera) inside it;
//   * its 2-sigma position ellipse on the ground;
//   * a label: track id, why it is shown, how old the newest detection is.
// The style says how to read it (overlay_geometry.hpp, Presence):
//   cyan    hidden from the device, agents see it now: the see-through case
//   green   the device has line of sight itself
//   grey    hidden, nobody sees it now: coasting on prediction, dashed
// and it fades as the data gets older. The optional truth ghost is a thin
// dashed magenta box.
//
// What is and is not ground truth here. The device's own pose comes from
// /sim/ground_truth: Phase 2 assumes a well-localised device, so overlay error
// is the track's error, not the device's. The true entity is used ONLY for
// ~/truth and for the optional ghost outline (`draw_truth`); nothing drawn for
// a track depends on it.

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>

#include <coop_msgs/msg/overlay_truth.hpp>
#include <coop_msgs/msg/track_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "device_view/overlay_geometry.hpp"
#include "synthetic_detector/geometry.hpp"
#include "synthetic_detector/sdf_occluders.hpp"

namespace device_view
{

namespace
{
double seconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

double yaw_of(const Quat & q)
{
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

cv::Point px(const Vec2 & p)
{
  return {static_cast<int>(std::lround(p.x())), static_cast<int>(std::lround(p.y()))};
}

void dashed_line(cv::Mat & img, const Vec2 & a, const Vec2 & b, const cv::Scalar & c, int thick)
{
  const double len = (b - a).norm();
  constexpr double kDash = 8.0;
  constexpr double kGap = 6.0;
  for (double s = 0.0; s < len; s += kDash + kGap) {
    const Vec2 p0 = a + (b - a) * (s / len);
    const Vec2 p1 = a + (b - a) * (std::min(s + kDash, len) / len);
    cv::line(img, px(p0), px(p1), c, thick, cv::LINE_AA);
  }
}

void segment(cv::Mat & img, const Vec2 & a, const Vec2 & b, const cv::Scalar & c, int thick, bool dashed)
{
  if (dashed) {
    dashed_line(img, a, b, c, thick);
  } else {
    cv::line(img, px(a), px(b), c, thick, cv::LINE_AA);
  }
}

// Text with a dark backing box, so it reads over any background.
void label(cv::Mat & img, const std::string & text, cv::Point at, const cv::Scalar & c)
{
  int base = 0;
  const auto sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.42, 1, &base);
  at.x = std::clamp(at.x, 2, std::max(2, img.cols - sz.width - 4));
  at.y = std::clamp(at.y, sz.height + 4, img.rows - 4);
  cv::rectangle(img, {at.x - 2, at.y - sz.height - 3}, {at.x + sz.width + 2, at.y + base},
    cv::Scalar(20, 20, 20), cv::FILLED);
  cv::putText(img, text, at, cv::FONT_HERSHEY_SIMPLEX, 0.42, c, 1, cv::LINE_AA);
}

// Magenta: unlike any track style, so the ghost never reads as a track.
const cv::Scalar kTruthColor(255, 0, 255);

constexpr std::array<std::array<int, 2>, 12> kBoxEdges{{
  {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}}};

struct Stamped
{
  double t;
  Vec3 p;
  Quat q;
};
}  // namespace

class OverlayNode : public rclcpp::Node
{
public:
  OverlayNode()
  : Node("overlay")
  {
    const auto image_topic = declare_parameter<std::string>("image_topic", "/device/camera/image");
    const auto info_topic = declare_parameter<std::string>("info_topic", "/device/camera/camera_info");
    const auto tracks_topic = declare_parameter<std::string>("tracks_topic", "/device/tracks");
    device_frame_ = declare_parameter<std::string>("device_frame", "device");
    entity_frames_ = declare_parameter<std::vector<std::string>>("entity_frames", {"entity"});
    entity_hist_.resize(entity_frames_.size());
    const auto mount = declare_parameter<std::vector<double>>("camera_mount_xyz", {0.0, 0.0, 1.6});
    mount_ = Vec3(mount.at(0), mount.at(1), mount.at(2));
    const auto size = declare_parameter<std::vector<double>>("entity_size_m", {0.5, 0.5, 1.75});
    size_ = Vec3(size.at(0), size.at(1), size.at(2));
    ref_height_ = declare_parameter<double>("entity_ref_height_m", 0.9);
    fade_s_ = declare_parameter<double>("fade_s", 4.0);
    match_radius_ = declare_parameter<double>("match_radius_m", 2.0);
    draw_truth_ = declare_parameter<bool>("draw_truth", true);
    video_path_ = declare_parameter<std::string>("video_path", "");
    video_fps_ = declare_parameter<double>("video_fps", 15.0);
    link_label_ = declare_parameter<std::string>("link_label", "perfect link");
    draw_hud_ = declare_parameter<bool>("draw_hud", true);  // off when the game view draws its own
    snapshot_dir_ = declare_parameter<std::string>("snapshot_dir", "");
    snapshot_every_s_ = declare_parameter<double>("snapshot_every_s", 2.0);

    const auto world_file = declare_parameter<std::string>("world_file", "");
    if (world_file.empty()) {
      throw std::invalid_argument("world_file is required (occluders: what hides the entity from the device)");
    }
    occluders_ = synthetic_detector::load_occluders(world_file,
        declare_parameter<std::vector<std::string>>("model_paths", std::vector<std::string>{}),
        declare_parameter<std::vector<std::string>>("exclude_models", {"entity"}));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CameraInfo & m) {on_info(m);});
    tracks_sub_ = create_subscription<coop_msgs::msg::TrackArray>(tracks_topic, 10,
        [this](coop_msgs::msg::TrackArray::ConstSharedPtr m) {tracks_ = m;});
    gt_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/sim/ground_truth", 50,
        [this](const tf2_msgs::msg::TFMessage & m) {on_ground_truth(m);});
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {on_image(m);});
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image", 5);
    truth_pub_ = create_publisher<coop_msgs::msg::OverlayTruth>("~/truth", 20);
    RCLCPP_INFO(get_logger(), "drawing %s on %s, %zu occluder(s)%s", tracks_topic.c_str(),
      image_topic.c_str(), occluders_.size(), video_path_.empty() ? "" : ", recording video");
  }

  ~OverlayNode() override
  {
    if (video_.isOpened()) {
      video_.release();
    }
  }

private:
  void on_info(const sensor_msgs::msg::CameraInfo & m)
  {
    cam_.fx = m.k[0];
    cam_.cx = m.k[2];
    cam_.fy = m.k[4];
    cam_.cy = m.k[5];
    cam_.width = static_cast<int>(m.width);
    cam_.height = static_cast<int>(m.height);
    have_info_ = cam_.fx > 0.0;
  }

  void on_ground_truth(const tf2_msgs::msg::TFMessage & m)
  {
    for (const auto & tf : m.transforms) {
      std::deque<Stamped> * hist = tf.child_frame_id == device_frame_ ? &device_hist_ : nullptr;
      for (std::size_t i = 0; hist == nullptr && i < entity_frames_.size(); ++i) {
        if (tf.child_frame_id == entity_frames_[i]) {
          hist = &entity_hist_[i];
        }
      }
      if (hist == nullptr) {
        continue;
      }
      const auto & tr = tf.transform.translation;
      const auto & r = tf.transform.rotation;
      hist->push_back({seconds(tf.header.stamp), Vec3(tr.x, tr.y, tr.z),
          Quat(r.w, r.x, r.y, r.z).normalized()});
      while (hist->size() > 200) {
        hist->pop_front();  // ~4 s at 50 Hz
      }
    }
  }

  static std::optional<Stamped> nearest(const std::deque<Stamped> & h, double t)
  {
    if (h.empty()) {
      return std::nullopt;
    }
    const auto it = std::min_element(h.begin(), h.end(),
        [t](const Stamped & a, const Stamped & b) {return std::abs(a.t - t) < std::abs(b.t - t);});
    if (std::abs(it->t - t) > 0.1) {
      return std::nullopt;  // no pose close enough to this frame
    }
    return *it;
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!have_info_) {
      return;
    }
    const double t = seconds(msg->header.stamp);
    const auto device = nearest(device_hist_, t);
    if (!device) {
      return;
    }
    const CameraPose pose = camera_on_device(device->p, yaw_of(device->q), mount_);

    cv_bridge::CvImagePtr cv = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat & img = cv->image;
    cv::Mat fill = img.clone();  // translucent fills go here, blended once at the end
    bool any_fill = false;

    // True entity boxes, for the evaluation messages and the ghost outlines.
    struct Truth
    {
      std::optional<Stamped> pose;
      std::optional<ProjectedBox> box;
      bool hidden{false};
      bool in_frustum{false};
    };
    std::vector<Truth> truths(entity_frames_.size());
    for (std::size_t i = 0; i < entity_frames_.size(); ++i) {
      Truth & tr = truths[i];
      tr.pose = nearest(entity_hist_[i], t);
      if (tr.pose) {
        tr.box = project_box(cam_, pose, box_corners(tr.pose->p, yaw_of(tr.pose->q), size_));
        tr.hidden = synthetic_detector::occluded(pose.p,
            tr.pose->p + Vec3(0.0, 0.0, ref_height_), occluders_);
        tr.in_frustum = tr.box && clip(tr.box->bounds, cam_).has_value();
      }
    }
    struct Drawn
    {
      std::uint32_t id;
      Vec3 ref;
      Rect bounds;
      int agents;
      double age;
    };
    std::vector<Drawn> drawn;

    int drawn_tracks = 0;
    if (tracks_) {
      const double dt = t - seconds(tracks_->stamp);
      for (const auto & tr : tracks_->tracks) {
        if (tr.status == coop_msgs::msg::Track::STATUS_TENTATIVE) {
          continue;
        }
        const Vec3 v(tr.velocity[0], tr.velocity[1], tr.velocity[2]);
        const Vec3 ref = Vec3(tr.position[0], tr.position[1], tr.position[2]) + dt * v;
        const Vec3 ground(ref.x(), ref.y(), ref.z() - ref_height_);
        const double age = t - seconds(tr.last_update);
        const bool hidden = synthetic_detector::occluded(pose.p, ref, occluders_);
        const int agents = std::popcount(static_cast<unsigned>(tr.source_mask));
        const Presence pr = presence(hidden, agents);

        // Facing: direction of travel when moving, else the last one seen.
        double yaw = std::atan2(pose.p.y() - ref.y(), pose.p.x() - ref.x());
        if (std::hypot(v.x(), v.y()) > 0.25) {
          yaw = std::atan2(v.y(), v.x());
          last_yaw_[tr.id] = yaw;
        } else if (const auto it = last_yaw_.find(tr.id); it != last_yaw_.end()) {
          yaw = it->second;
        }
        const auto box = project_box(cam_, pose, box_corners(ground, yaw, size_));
        if (!box || !clip(box->bounds, cam_)) {
          continue;  // behind the device, or outside the image
        }

        const double a = freshness(age, fade_s_, 0.25);
        cv::Scalar color;
        std::string why;
        switch (pr) {
          case Presence::kSeenThroughWall:
            color = cv::Scalar(255, 255, 0);
            why = "behind wall, " + std::to_string(agents) + (agents == 1 ? " agent" : " agents");
            break;
          case Presence::kVisibleToDevice:
            color = cv::Scalar(90, 220, 90);
            why = "in view";
            break;
          case Presence::kCoasting:
            color = cv::Scalar(170, 170, 170);
            why = "predicted";
            break;
        }
        color *= a;
        const bool dashed = pr == Presence::kCoasting;
        const int thick = pr == Presence::kSeenThroughWall ? 2 : 1;

        // 2-sigma ground ellipse.
        Mat2 cov;
        cov << tr.covariance[0], tr.covariance[1], tr.covariance[1], tr.covariance[3];
        const auto ell = ground_ellipse(ground.head<2>(), cov, 2.0, ground.z());
        for (std::size_t i = 0; i < ell.size(); ++i) {
          const auto p0 = project(cam_, pose, ell[i]);
          const auto p1 = project(cam_, pose, ell[(i + 1) % ell.size()]);
          if (p0 && p1) {
            segment(img, *p0, *p1, color, 1, true);
          }
        }
        // Box.
        for (const auto & e : kBoxEdges) {
          segment(img, box->corners[static_cast<std::size_t>(e[0])],
            box->corners[static_cast<std::size_t>(e[1])], color, thick, dashed);
        }
        // Silhouette: translucent fill plus outline.
        const Silhouette sil = person_silhouette(ground, size_.z(), pose.p);
        std::vector<cv::Point> body;
        std::vector<cv::Point> head;
        for (const auto & p : sil.body) {
          if (const auto q = project(cam_, pose, p)) {body.push_back(px(*q));}
        }
        for (const auto & p : sil.head) {
          if (const auto q = project(cam_, pose, p)) {head.push_back(px(*q));}
        }
        if (body.size() == sil.body.size() && head.size() == sil.head.size()) {
          if (pr != Presence::kCoasting) {
            cv::fillPoly(fill, std::vector<std::vector<cv::Point>>{body, head}, color, cv::LINE_AA);
            any_fill = true;
          }
          cv::polylines(img, std::vector<std::vector<cv::Point>>{body, head}, true, color, thick,
            cv::LINE_AA);
        }
        char age_txt[32];
        std::snprintf(age_txt, sizeof(age_txt), "%.1f s", std::max(age, 0.0));
        label(img, "T" + std::to_string(tr.id) + "  " + why + "  " + age_txt,
          {static_cast<int>(box->bounds.x0), static_cast<int>(box->bounds.y0) - 6}, color);
        ++drawn_tracks;

        drawn.push_back({tr.id, ref, box->bounds, agents, age});
      }
    }
    if (any_fill) {
      cv::addWeighted(fill, 0.35, img, 0.65, 0.0, img);
    }
    // Evaluation: for each entity, the nearest drawn track within the match radius.
    for (std::size_t i = 0; i < truths.size(); ++i) {
      const Truth & tr = truths[i];
      coop_msgs::msg::OverlayTruth ot;
      ot.stamp = msg->header.stamp;
      ot.entity = entity_frames_[i];
      ot.in_frustum = tr.in_frustum;
      ot.hidden = tr.hidden;
      if (tr.pose) {
        double best = match_radius_;
        for (const Drawn & d : drawn) {
          const double dist = std::hypot(d.ref.x() - tr.pose->p.x(), d.ref.y() - tr.pose->p.y());
          if (dist <= best) {
            best = dist;
            ot.drawn = true;
            ot.track_id = d.id;
            ot.agents_seeing = static_cast<std::uint8_t>(d.agents);
            ot.age_s = static_cast<float>(d.age);
            ot.world_error_m = static_cast<float>(dist);
            if (tr.box) {
              ot.pixel_error = static_cast<float>((d.bounds.center() - tr.box->bounds.center()).norm());
              ot.iou = static_cast<float>(iou(d.bounds, tr.box->bounds));
            }
          }
        }
      }
      truth_pub_->publish(ot);
      if (draw_truth_ && tr.box && tr.in_frustum) {
        for (const auto & e : kBoxEdges) {
          dashed_line(img, tr.box->corners[static_cast<std::size_t>(e[0])],
            tr.box->corners[static_cast<std::size_t>(e[1])], kTruthColor, 1);
        }
      }
    }
    if (draw_hud_) {
      draw_hud(img, t, drawn_tracks);
    }
    image_pub_->publish(*cv->toImageMsg());
    write_video(img);
    write_snapshot(img, t);
  }

  void write_snapshot(const cv::Mat & img, double t)
  {
    if (snapshot_dir_.empty() || t < next_snapshot_t_) {
      return;
    }
    next_snapshot_t_ = t + snapshot_every_s_;
    char name[64];
    std::snprintf(name, sizeof(name), "/frame_%07.2f.png", t);
    cv::imwrite(snapshot_dir_ + name, img);
  }

  void draw_hud(cv::Mat & img, double t, int n)
  {
    char head[96];
    std::snprintf(head, sizeof(head), "device view  t=%.1f s  %d track%s  %s", t, n,
      n == 1 ? "" : "s", link_label_.c_str());
    label(img, head, {8, 18}, cv::Scalar(235, 235, 235));
    int y = img.rows - 58;
    const std::array<std::pair<const char *, cv::Scalar>, 4> legend{{
      {"behind wall, seen by agents", cv::Scalar(255, 255, 0)},
      {"in the device's own view", cv::Scalar(90, 220, 90)},
      {"predicted, nobody sees it", cv::Scalar(170, 170, 170)},
      {"truth (simulation only)", kTruthColor}}};
    for (const auto & [text, color] : legend) {
      if (!draw_truth_ && color == kTruthColor) {
        continue;
      }
      label(img, text, {8, y}, color);
      y += 15;
    }
  }

  void write_video(const cv::Mat & img)
  {
    if (video_path_.empty()) {
      return;
    }
    if (!video_.isOpened()) {
      video_.open(video_path_, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), video_fps_, img.size());
      if (!video_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "cannot open %s for writing; not recording", video_path_.c_str());
        video_path_.clear();
        return;
      }
    }
    video_.write(img);
  }

  // Configuration
  std::string device_frame_;
  std::vector<std::string> entity_frames_;
  Vec3 mount_{Vec3::Zero()};
  Vec3 size_{Vec3::Zero()};
  double ref_height_{0.9};
  double fade_s_{4.0};
  double match_radius_{2.0};
  bool draw_truth_{true};
  std::string video_path_;
  double video_fps_{15.0};
  std::string link_label_;
  bool draw_hud_{true};
  std::string snapshot_dir_;
  double snapshot_every_s_{2.0};
  double next_snapshot_t_{0.0};
  synthetic_detector::Occluders occluders_;

  // State (single-threaded executor)
  PinholeCamera cam_;
  bool have_info_{false};
  coop_msgs::msg::TrackArray::ConstSharedPtr tracks_;
  std::deque<Stamped> device_hist_;
  std::vector<std::deque<Stamped>> entity_hist_;
  std::map<std::uint32_t, double> last_yaw_;
  cv::VideoWriter video_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<coop_msgs::msg::TrackArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr gt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<coop_msgs::msg::OverlayTruth>::SharedPtr truth_pub_;
};

}  // namespace device_view

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<device_view::OverlayNode>());
  rclcpp::shutdown();
  return 0;
}
