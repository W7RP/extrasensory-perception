// The playable view: you are the ground device.
//
// A window (SDL2) showing the device's see-through camera view full size, a
// minimap with fog of war in the corner, and a small HUD. The keyboard and
// mouse drive the device:
//
//   W / S, Up / Down     walk forward / back
//   A / D                step left / right
//   Q / E, Left / Right  turn;   mouse: turn while the right button is held
//                        (or all the time after Tab captures the mouse)
//   Shift                run
//   M  minimap   F  fog of war   H  help   Esc  quit (ends the session)
//
//   in   /device/overlay/image, /device/tracks, /device/status,
//        /agent_<n>/status (what each agent reports about itself)
//        with a zoom camera on agent z (`zoom_agent`, Phase 4): its video
//        (`zoom_image_topic`) and /agent_<z>/gimbal/state, shown as a
//        picture-in-picture with what the device can work out itself: the
//        track it is on, slant range, ground resolution, and the Johnson
//        level (detect / recognise / identify) for a person at that range
//   out  /device/device_controller/cmd (geometry_msgs/Twist, 30 Hz)
//        ~/image   the composed frame, 10 Hz (recording, and checking it without a screen)
//        /game/quit (std_msgs/Bool, latched) when the player quits
//
// `demo_input` replays a key sequence instead of reading the keyboard
// ("w:3,q:1.5,w:2": keys, seconds; R = run, any other letter = idle), so the
// whole path can be exercised without a person; `window: false` runs it
// without opening a window. `video_path` records what the window shows, and
// `snapshot_dir` keeps a PNG every 3 s.
//
// Threads: SDL has to own the main thread, so ROS spins on a second one; the
// two share the latest inputs under one mutex.

#include <SDL2/SDL.h>

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>

#include <coop_msgs/msg/agent_status.hpp>
#include <coop_msgs/msg/device_status.hpp>
#include <coop_msgs/msg/gimbal_state.hpp>
#include <coop_msgs/msg/track_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include "device_view/minimap.hpp"
#include "device_view/walker.hpp"
#include "synthetic_detector/imaging.hpp"
#include "synthetic_detector/sdf_occluders.hpp"

namespace device_view
{

namespace
{
using synthetic_detector::Quat;
using synthetic_detector::Vec3;
double deg(double d) {return d * M_PI / 180.0;}

// Text on a dark backing box, readable over sky, walls and grass alike.
void hud_text(cv::Mat & img, const std::string & text, cv::Point at, const cv::Scalar & c,
  double scale = 0.55)
{
  int base = 0;
  const auto sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &base);
  cv::rectangle(img, {at.x - 4, at.y - sz.height - 5}, {at.x + sz.width + 4, at.y + base + 2},
    cv::Scalar(18, 18, 18), cv::FILLED);
  cv::putText(img, text, at, cv::FONT_HERSHEY_SIMPLEX, scale, c, 1, cv::LINE_AA);
}

// "w:3,q:1.5" -> [(w, 3.0), (q, 1.5)]
std::vector<std::pair<std::string, double>> parse_demo(const std::string & s)
{
  std::vector<std::pair<std::string, double>> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto colon = item.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    out.emplace_back(item.substr(0, colon), std::stod(item.substr(colon + 1)));
  }
  return out;
}
}  // namespace

class GameViewNode : public rclcpp::Node
{
public:
  GameViewNode()
  : Node("game_view")
  {
    width_ = static_cast<int>(declare_parameter<int>("window_width", 1280));
    height_ = static_cast<int>(declare_parameter<int>("window_height", 960));
    window_ = declare_parameter<bool>("window", true);
    demo_ = parse_demo(declare_parameter<std::string>("demo_input", ""));
    mouse_sens_ = declare_parameter<double>("mouse_turn_per_px", 0.06);
    video_path_ = declare_parameter<std::string>("video_path", "");
    snapshot_dir_ = declare_parameter<std::string>("snapshot_dir", "");
    const auto agents = declare_parameter<std::vector<std::int64_t>>("agents", {1, 2});
    const auto b = declare_parameter<std::vector<double>>("bounds_xy", {-30.0, 30.0, -30.0, 30.0});
    map_.xmin = b.at(0);
    map_.xmax = b.at(1);
    map_.ymin = b.at(2);
    map_.ymax = b.at(3);
    map_.cell_m = declare_parameter<double>("fog_cell_m", 1.0);
    device_cam_.hfov_rad = deg(declare_parameter<double>("device_hfov_deg", 70.0));
    device_cam_.vfov_rad = deg(declare_parameter<double>("device_vfov_deg", 55.0));
    device_cam_.pitch_down_rad = 0.0;
    device_cam_.max_range_m = declare_parameter<double>("device_view_range_m", 60.0);
    device_cam_.mount_offset_body = Vec3(0.0, 0.0, declare_parameter<double>("device_camera_height_m", 1.6));
    occ_ = synthetic_detector::load_occluders(declare_parameter<std::string>("world_file", ""),
        declare_parameter<std::vector<std::string>>("model_paths", std::vector<std::string>{}),
        declare_parameter<std::vector<std::string>>("exclude_models", {"entity"}));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>("/device/overlay/image", 5,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          auto cv = cv_bridge::toCvCopy(m, "bgr8");
          std::lock_guard lock(mutex_);
          frame_ = cv->image;
        });
    tracks_sub_ = create_subscription<coop_msgs::msg::TrackArray>("/device/tracks", 5,
        [this](coop_msgs::msg::TrackArray::ConstSharedPtr m) {
          std::lock_guard lock(mutex_);
          tracks_ = m;
        });
    device_sub_ = create_subscription<coop_msgs::msg::DeviceStatus>("/device/status", 5,
        [this](const coop_msgs::msg::DeviceStatus & m) {
          std::lock_guard lock(mutex_);
          device_ = m;
          have_device_ = true;
        });
    for (const auto a : agents) {
      agent_subs_.push_back(create_subscription<coop_msgs::msg::AgentStatus>(
          "/agent_" + std::to_string(a) + "/status", 5,
          [this](const coop_msgs::msg::AgentStatus & m) {
            std::lock_guard lock(mutex_);
            agents_[m.agent] = m;
          }));
    }
    zoom_agent_ = static_cast<int>(declare_parameter<int>("zoom_agent", 0));
    if (zoom_agent_ > 0) {
      zoom_sub_ = create_subscription<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("zoom_image_topic", "/agent_1/zoom/image"), 2,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          auto cv = cv_bridge::toCvCopy(m, "bgr8");
          std::lock_guard lock(mutex_);
          zoom_frame_ = cv->image;
        });
      gimbal_sub_ = create_subscription<coop_msgs::msg::GimbalState>(
        "/agent_" + std::to_string(zoom_agent_) + "/gimbal/state", 5,
        [this](const coop_msgs::msg::GimbalState & m) {
          std::lock_guard lock(mutex_);
          gimbal_ = m;
          have_gimbal_ = true;
        });
    }
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/device/device_controller/cmd", 10);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image", 2);
    quit_pub_ = create_publisher<std_msgs::msg::Bool>("/game/quit",
        rclcpp::QoS(1).reliable().transient_local());
  }

  int run()
  {
    SDL_Window * win = nullptr;
    SDL_Renderer * ren = nullptr;
    SDL_Texture * tex = nullptr;
    if (window_) {
      if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        RCLCPP_ERROR(get_logger(), "SDL_Init failed: %s; running without a window", SDL_GetError());
        window_ = false;
      } else {
        win = SDL_CreateWindow("coop-perception-sim: device view", SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED, width_, height_, SDL_WINDOW_SHOWN);
        ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, width_,
            height_);
      }
    }
    const auto t_start = std::chrono::steady_clock::now();
    auto last_cmd = t_start;
    auto last_pub = t_start;
    auto last_fog = t_start - std::chrono::seconds(1);
    auto last_frame = t_start;
    auto last_video = t_start;
    auto last_snap = t_start;
    bool captured = false;
    int fps_frames = 0;
    double fps = 0.0;
    auto fps_t = t_start;

    while (rclcpp::ok() && !quit_) {
      const auto now_w = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now_w - last_frame).count();
      last_frame = now_w;
      // ---- input
      WalkCommand cmd;
      double mouse_turn = 0.0;
      if (window_) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
          if (e.type == SDL_QUIT) {
            quit_ = true;
          } else if (e.type == SDL_KEYDOWN && !e.key.repeat) {
            switch (e.key.keysym.sym) {
              case SDLK_ESCAPE: quit_ = true; break;
              case SDLK_m: show_map_ = !show_map_; break;
              case SDLK_f: show_fog_ = !show_fog_; break;
              case SDLK_h: show_help_ = !show_help_; break;
              case SDLK_TAB:
                captured = !captured;
                SDL_SetRelativeMouseMode(captured ? SDL_TRUE : SDL_FALSE);
                break;
              default: break;
            }
          } else if (e.type == SDL_MOUSEMOTION &&
            (captured || (e.motion.state & SDL_BUTTON_RMASK)))
          {
            mouse_turn -= e.motion.xrel * mouse_sens_;
          }
        }
        const Uint8 * k = SDL_GetKeyboardState(nullptr);
        cmd.forward = (k[SDL_SCANCODE_W] || k[SDL_SCANCODE_UP]) - (k[SDL_SCANCODE_S] || k[SDL_SCANCODE_DOWN]);
        cmd.left = k[SDL_SCANCODE_A] - k[SDL_SCANCODE_D];
        cmd.turn = (k[SDL_SCANCODE_Q] || k[SDL_SCANCODE_LEFT]) - (k[SDL_SCANCODE_E] || k[SDL_SCANCODE_RIGHT]);
        cmd.run = k[SDL_SCANCODE_LSHIFT] || k[SDL_SCANCODE_RSHIFT];
      }
      if (!demo_.empty()) {
        const double t = std::chrono::duration<double>(now_w - t_start).count();
        double acc = 0.0;
        cmd = WalkCommand{};
        bool done = true;
        for (const auto & [key, secs] : demo_) {
          if (t < acc + secs) {
            for (char c : key) {
              cmd.forward += c == 'w' ? 1.0 : c == 's' ? -1.0 : 0.0;
              cmd.left += c == 'a' ? 1.0 : c == 'd' ? -1.0 : 0.0;
              cmd.turn += c == 'q' ? 1.0 : c == 'e' ? -1.0 : 0.0;
              cmd.run = cmd.run || c == 'R';
            }
            done = false;
            break;
          }
          acc += secs;
        }
        if (done) {
          quit_ = true;  // a scripted session ends with its input
        }
      }
      cmd.turn = std::clamp(cmd.turn + mouse_turn / std::max(dt, 1e-3) / 60.0, -1.0, 1.0);
      if (now_w - last_cmd >= std::chrono::milliseconds(33)) {
        geometry_msgs::msg::Twist tw;
        tw.linear.x = cmd.forward;
        tw.linear.y = cmd.left;
        tw.linear.z = cmd.run ? 1.0 : 0.0;
        tw.angular.z = cmd.turn;
        cmd_pub_->publish(tw);
        last_cmd = now_w;
      }

      // ---- compose
      cv::Mat frame;
      coop_msgs::msg::TrackArray::ConstSharedPtr tracks;
      coop_msgs::msg::DeviceStatus dev;
      std::vector<coop_msgs::msg::AgentStatus> agents;
      bool have_device = false;
      cv::Mat zoom_frame;
      coop_msgs::msg::GimbalState gimbal;
      bool have_gimbal = false;
      {
        std::lock_guard lock(mutex_);
        if (!frame_.empty()) {
          frame = frame_;
        }
        zoom_frame = zoom_frame_;
        gimbal = gimbal_;
        have_gimbal = have_gimbal_;
        tracks = tracks_;
        dev = device_;
        have_device = have_device_;
        for (const auto & [id, a] : agents_) {
          agents.push_back(a);
        }
      }
      cv::Mat view(height_, width_, CV_8UC3, cv::Scalar(20, 20, 20));
      if (!frame.empty()) {
        cv::resize(frame, view, view.size(), 0, 0, cv::INTER_LINEAR);
      } else {
        cv::putText(view, "waiting for the device camera...", {40, height_ / 2},
          cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(220, 220, 220), 2, cv::LINE_AA);
      }
      if (have_device && show_map_) {
        Viewer device;
        device.body.p = Vec3(dev.position[0], dev.position[1], dev.position[2]);
        device.body.q = Quat(Eigen::AngleAxisd(dev.yaw, Vec3::UnitZ()));
        device.camera = device_cam_;
        std::vector<Viewer> viewers;
        for (const auto & a : agents) {
          Viewer v;
          v.body.p = Vec3(a.position[0], a.position[1], a.position[2]);
          v.body.q = Quat(Eigen::AngleAxisd(a.yaw, Vec3::UnitZ()));
          v.camera.hfov_rad = a.hfov;
          v.camera.vfov_rad = a.vfov;
          v.camera.pitch_down_rad = a.camera_pitch_down;
          v.camera.max_range_m = a.max_range_m;
          v.camera.mount_offset_body = Vec3::Zero();
          viewers.push_back(v);
        }
        if (show_fog_ && now_w - last_fog > std::chrono::milliseconds(250)) {
          fog_ = fog_of_war(map_, device, viewers, occ_);
          last_fog = now_w;
        }
        std::vector<MapTrack> mt;
        if (tracks) {
          for (const auto & t : tracks->tracks) {
            if (t.status == coop_msgs::msg::Track::STATUS_TENTATIVE) {
              continue;
            }
            mt.push_back({t.position[0], t.position[1], t.id,
                std::popcount(static_cast<unsigned>(t.source_mask)),
                t.status == coop_msgs::msg::Track::STATUS_COASTING,
                have_gimbal && gimbal.track_id != 0 && gimbal.track_id == t.id});
          }
        }
        const int size = std::min(width_, height_) * 3 / 8;
        const cv::Mat map = draw_minimap(size, map_, occ_, show_fog_ ? fog_ : std::vector<CellView>{},
            device, viewers, mt);
        const cv::Rect roi(width_ - size - 12, height_ - size - 12, size, size);
        cv::addWeighted(map, 0.88, view(roi), 0.12, 0.0, view(roi));
      }
      if (zoom_agent_ > 0) {
        draw_zoom(view, zoom_frame, have_gimbal ? &gimbal : nullptr, tracks, agents);
      }
      // HUD: status top left, legend under it, help bottom left, fps top right.
      int held = 0;
      int live = 0;
      if (tracks) {
        for (const auto & t : tracks->tracks) {
          if (t.status == coop_msgs::msg::Track::STATUS_TENTATIVE) {
            continue;
          }
          ++held;
          live += t.status == coop_msgs::msg::Track::STATUS_CONFIRMED;
        }
      }
      char status[128];
      std::snprintf(status, sizeof(status), "%d tracked, %d seen right now   |   %zu agents   |   perfect link",
        held, live, agents.size());
      hud_text(view, status, {14, 28}, cv::Scalar(240, 240, 240));
      const std::array<std::pair<const char *, cv::Scalar>, 3> legend{{
        {"behind a wall, an agent sees it now", cv::Scalar(255, 255, 0)},
        {"in your own line of sight", cv::Scalar(90, 220, 90)},
        {"predicted: nobody sees it now", cv::Scalar(170, 170, 170)}}};
      int ly = 56;
      for (const auto & [text, color] : legend) {
        hud_text(view, text, {14, ly}, color, 0.48);
        ly += 22;
      }
      ++fps_frames;
      if (std::chrono::duration<double>(now_w - fps_t).count() > 1.0) {
        fps = fps_frames / std::chrono::duration<double>(now_w - fps_t).count();
        fps_frames = 0;
        fps_t = now_w;
      }
      char fps_txt[32];
      std::snprintf(fps_txt, sizeof(fps_txt), "%.0f fps", fps);
      hud_text(view, fps_txt, {width_ - 86, 28}, cv::Scalar(220, 220, 220), 0.48);
      if (show_help_) {
        const std::vector<std::string> help{
          "WASD walk   Q/E or right mouse: turn   Shift run",
          "Tab capture mouse   M map   F fog   H hide help   Esc end"};
        int y = height_ - 42;
        for (const auto & line : help) {
          hud_text(view, line, {14, y}, cv::Scalar(235, 235, 235), 0.48);
          y += 22;
        }
      }

      if (window_) {
        SDL_UpdateTexture(tex, nullptr, view.data, static_cast<int>(view.step));
        SDL_RenderClear(ren);
        SDL_RenderCopy(ren, tex, nullptr, nullptr);
        SDL_RenderPresent(ren);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
      }
      if (!video_path_.empty() && now_w - last_video >= std::chrono::milliseconds(66)) {
        if (!video_.isOpened()) {
          video_.open(video_path_, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 15.0, view.size());
        }
        if (video_.isOpened()) {
          video_.write(view);
        }
        last_video = now_w;
      }
      if (!snapshot_dir_.empty() && now_w - last_snap >= std::chrono::seconds(3)) {
        char name[64];
        std::snprintf(name, sizeof(name), "/game_%06.1f.png",
          std::chrono::duration<double>(now_w - t_start).count());
        cv::imwrite(snapshot_dir_ + name, view);
        last_snap = now_w;
      }
      if (now_w - last_pub >= std::chrono::milliseconds(100)) {
        std_msgs::msg::Header h;
        h.stamp = get_clock()->now();
        image_pub_->publish(*cv_bridge::CvImage(h, "bgr8", view).toImageMsg());
        last_pub = now_w;
      }
    }
    std_msgs::msg::Bool q;
    q.data = true;
    quit_pub_->publish(q);
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    if (video_.isOpened()) {
      video_.release();
    }
    if (window_) {
      SDL_DestroyTexture(tex);
      SDL_DestroyRenderer(ren);
      SDL_DestroyWindow(win);
      SDL_Quit();
    }
    return 0;
  }

private:
  // The zoom camera's picture-in-picture, top right.
  void draw_zoom(
    cv::Mat & view, const cv::Mat & zoom, const coop_msgs::msg::GimbalState * g,
    const coop_msgs::msg::TrackArray::ConstSharedPtr & tracks,
    const std::vector<coop_msgs::msg::AgentStatus> & agents) const
  {
    const int w = width_ * 34 / 100;
    const int h = w * 9 / 16;
    const cv::Rect roi(width_ - w - 12, 44, w, h);
    if (zoom.empty()) {
      cv::rectangle(view, roi, cv::Scalar(20, 20, 20), cv::FILLED);
      hud_text(view, "zoom camera: no video yet", {roi.x + 10, roi.y + h / 2}, cv::Scalar(200, 200, 200), 0.5);
    } else {
      cv::resize(zoom, view(roi), roi.size(), 0, 0, cv::INTER_AREA);
    }
    const cv::Point c(roi.x + w / 2, roi.y + h / 2);
    const cv::Scalar reticle(60, 140, 255);
    for (const auto & [dx, dy] : std::array<std::array<int, 2>, 4>{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
      cv::line(view, c + cv::Point(dx * 8, dy * 8), c + cv::Point(dx * 22, dy * 22), reticle, 1, cv::LINE_AA);
    }
    cv::rectangle(view, roi, cv::Scalar(200, 200, 200), 1);

    std::string label = "ZOOM  stowed";
    std::string detail;
    const synthetic_detector::SensorSpec * spec = synthetic_detector::sensor_preset("eo_zoom_30x");
    const auto agent = std::find_if(agents.begin(), agents.end(),
        [this](const auto & a) {return a.agent == zoom_agent_;});
    const coop_msgs::msg::Track * target = nullptr;
    if (g && tracks && g->track_id != 0) {
      for (const auto & t : tracks->tracks) {
        if (t.id == g->track_id) {
          target = &t;
        }
      }
    }
    if (target && agent != agents.end() && spec) {
      // What the device can work out from reports alone: the agent's own
      // position, the track's, and the camera's spec.
      const Vec3 d(target->position[0] - agent->position[0], target->position[1] - agent->position[1],
        target->position[2] - agent->position[2]);
      const double range = d.norm();
      const double depression = std::atan2(-d.z(), std::hypot(d.x(), d.y()));
      const double gsd = range * g->hfov / spec->width_px;
      const double px = synthetic_detector::pixels_on_target(*spec, range,
          synthetic_detector::critical_dimension_m(0.5, 0.3, 1.75, depression));
      static constexpr std::array<const char *, 4> kLevel{"below detection", "detect", "recognise",
        "identify"};
      char b[96];
      std::snprintf(b, sizeof(b), "ZOOM  track %u  %s", target->id,
        g->on_target ? "on target" : "slewing");
      label = b;
      std::snprintf(b, sizeof(b), "%.0f m   %.1f cm/px   %.0f px: %s", range, 100.0 * gsd, px,
        kLevel[static_cast<std::size_t>(synthetic_detector::johnson_level(px))]);
      detail = b;
      // A person at that range, to scale, around the reticle.
      const double scale = w / (range * g->hfov);  // view pixels per metre at the target
      const int pw = std::max(2, static_cast<int>(std::lround(0.5 * scale)));
      const int ph = std::max(3, static_cast<int>(std::lround(
          (0.3 * std::sin(depression) + 1.75 * std::cos(depression)) * scale)));
      cv::rectangle(view, {c.x - pw / 2, c.y - ph / 2}, {c.x + pw / 2, c.y + ph / 2}, reticle, 1,
        cv::LINE_AA);
    } else if (g && g->track_id != 0) {
      label = "ZOOM  track " + std::to_string(g->track_id) + "  slewing";
    }
    hud_text(view, label, {roi.x + 8, roi.y + h - (detail.empty() ? 10 : 32)}, reticle, 0.45);
    if (!detail.empty()) {
      hud_text(view, detail, {roi.x + 8, roi.y + h - 10}, reticle, 0.45);
    }
  }

  int zoom_agent_{0};
  cv::Mat zoom_frame_;
  coop_msgs::msg::GimbalState gimbal_;
  bool have_gimbal_{false};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr zoom_sub_;
  rclcpp::Subscription<coop_msgs::msg::GimbalState>::SharedPtr gimbal_sub_;

  int width_{1280};
  int height_{960};
  bool window_{true};
  std::vector<std::pair<std::string, double>> demo_;
  double mouse_sens_{0.06};
  std::string video_path_;
  std::string snapshot_dir_;
  cv::VideoWriter video_;
  MinimapConfig map_;
  synthetic_detector::CameraConfig device_cam_;
  synthetic_detector::Occluders occ_;
  std::vector<CellView> fog_;
  bool show_map_{true};
  bool show_fog_{true};
  bool show_help_{true};
  std::atomic<bool> quit_{false};

  std::mutex mutex_;
  cv::Mat frame_;
  coop_msgs::msg::TrackArray::ConstSharedPtr tracks_;
  coop_msgs::msg::DeviceStatus device_;
  bool have_device_{false};
  std::map<std::uint8_t, coop_msgs::msg::AgentStatus> agents_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<coop_msgs::msg::TrackArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<coop_msgs::msg::DeviceStatus>::SharedPtr device_sub_;
  std::vector<rclcpp::Subscription<coop_msgs::msg::AgentStatus>::SharedPtr> agent_subs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr quit_pub_;
};

}  // namespace device_view

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<device_view::GameViewNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread ros([&exec]() {exec.spin();});
  const int rc = node->run();
  exec.cancel();
  ros.join();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
