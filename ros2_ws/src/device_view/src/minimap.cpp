#include "device_view/minimap.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <opencv2/imgproc.hpp>

namespace device_view
{

namespace
{
using synthetic_detector::Vec3;

struct ToPx
{
  const MinimapConfig & cfg;
  double scale;
  cv::Point operator()(double x, double y) const
  {
    return {static_cast<int>(std::lround((x - cfg.xmin) * scale)),
      static_cast<int>(std::lround((cfg.ymax - y) * scale))};
  }
};

double yaw_of(const synthetic_detector::Quat & q)
{
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

// Field-of-view wedge on the ground, out to the camera's range.
void wedge(cv::Mat & img, const ToPx & to, const Viewer & v, const cv::Scalar & c)
{
  const double yaw = yaw_of(v.body.q);
  const double r = v.camera.max_range_m;
  const double h = 0.5 * v.camera.hfov_rad;
  std::vector<cv::Point> pts{to(v.body.p.x(), v.body.p.y())};
  for (int i = 0; i <= 12; ++i) {
    const double a = yaw - h + 2.0 * h * i / 12.0;
    pts.push_back(to(v.body.p.x() + r * std::cos(a), v.body.p.y() + r * std::sin(a)));
  }
  cv::polylines(img, std::vector<std::vector<cv::Point>>{pts}, true, c, 1, cv::LINE_AA);
}
}  // namespace

std::vector<CellView> fog_of_war(
  const MinimapConfig & cfg, const Viewer & device, std::span<const Viewer> agents,
  const synthetic_detector::Occluders & occ)
{
  const int nx = static_cast<int>(std::ceil((cfg.xmax - cfg.xmin) / cfg.cell_m));
  const int ny = static_cast<int>(std::ceil((cfg.ymax - cfg.ymin) / cfg.cell_m));
  std::vector<CellView> fog(static_cast<std::size_t>(nx * ny), CellView::kNobody);
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const Vec3 p(cfg.xmin + (i + 0.5) * cfg.cell_m, cfg.ymin + (j + 0.5) * cfg.cell_m,
        cfg.probe_height_m);
      CellView v = CellView::kNobody;
      if (synthetic_detector::evaluate_visibility(device.body, p, device.camera, occ).visible()) {
        v = CellView::kDevice;
      } else {
        for (const auto & a : agents) {
          if (synthetic_detector::evaluate_visibility(a.body, p, a.camera, occ).visible()) {
            v = CellView::kAgent;
            break;
          }
        }
      }
      fog[static_cast<std::size_t>(j * nx + i)] = v;
    }
  }
  return fog;
}

cv::Mat draw_minimap(
  int size_px, const MinimapConfig & cfg, const synthetic_detector::Occluders & occ,
  const std::vector<CellView> & fog, const Viewer & device, std::span<const Viewer> agents,
  std::span<const MapTrack> tracks)
{
  const double span_m = std::max(cfg.xmax - cfg.xmin, cfg.ymax - cfg.ymin);
  const ToPx to{cfg, size_px / span_m};
  cv::Mat img(size_px, size_px, CV_8UC3, cv::Scalar(28, 34, 30));

  // Fog: nobody = dark, agents only = cyan-tinted, device = light.
  const int nx = static_cast<int>(std::ceil((cfg.xmax - cfg.xmin) / cfg.cell_m));
  const int ny = static_cast<int>(std::ceil((cfg.ymax - cfg.ymin) / cfg.cell_m));
  if (fog.size() == static_cast<std::size_t>(nx * ny)) {
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {
        const CellView v = fog[static_cast<std::size_t>(j * nx + i)];
        if (v == CellView::kNobody) {
          continue;
        }
        const double x0 = cfg.xmin + i * cfg.cell_m;
        const double y0 = cfg.ymin + j * cfg.cell_m;
        cv::rectangle(img, to(x0, y0 + cfg.cell_m), to(x0 + cfg.cell_m, y0),
          v == CellView::kDevice ? cv::Scalar(92, 110, 96) : cv::Scalar(110, 92, 40), cv::FILLED);
      }
    }
  }

  // The map: building and crate footprints, trunks, canopies.
  for (const auto & b : occ.boxes) {
    std::vector<cv::Point> pts;
    for (const auto & [sx, sy] : std::array<std::array<double, 2>, 4>{{{1, 1}, {-1, 1}, {-1, -1}, {1, -1}}}) {
      const Vec3 c = b.center + b.q * Vec3(sx * b.half_extents.x(), sy * b.half_extents.y(), 0.0);
      pts.push_back(to(c.x(), c.y()));
    }
    cv::fillPoly(img, std::vector<std::vector<cv::Point>>{pts}, cv::Scalar(120, 130, 150), cv::LINE_AA);
    cv::polylines(img, std::vector<std::vector<cv::Point>>{pts}, true, cv::Scalar(170, 180, 200), 1,
      cv::LINE_AA);
  }
  for (const auto & s : occ.spheres) {
    cv::circle(img, to(s.center.x(), s.center.y()), std::max(2, static_cast<int>(s.radius * to.scale)),
      cv::Scalar(60, 120, 60), cv::FILLED, cv::LINE_AA);
  }
  for (const auto & c : occ.cylinders) {
    cv::circle(img, to(c.center.x(), c.center.y()), std::max(1, static_cast<int>(c.radius * to.scale)),
      cv::Scalar(40, 70, 110), cv::FILLED, cv::LINE_AA);
  }

  // Agents: position, view wedge, id.
  for (const auto & a : agents) {
    wedge(img, to, a, cv::Scalar(200, 170, 60));
    cv::circle(img, to(a.body.p.x(), a.body.p.y()), 5, cv::Scalar(255, 200, 80), cv::FILLED,
      cv::LINE_AA);
  }
  // Tracks: cyan when an agent sees it, grey when predicted.
  for (const auto & t : tracks) {
    const cv::Scalar c = t.coasting ? cv::Scalar(150, 150, 150) :
      (t.agents_seeing > 0 ? cv::Scalar(255, 255, 0) : cv::Scalar(90, 220, 90));
    const cv::Point p = to(t.x, t.y);
    cv::circle(img, p, 5, c, t.coasting ? 1 : cv::FILLED, cv::LINE_AA);
    cv::putText(img, std::to_string(t.id), p + cv::Point(6, -4), cv::FONT_HERSHEY_SIMPLEX, 0.35,
      c, 1, cv::LINE_AA);
  }
  // The device: an arrow and its view wedge.
  wedge(img, to, device, cv::Scalar(120, 230, 120));
  const double yaw = yaw_of(device.body.q);
  const cv::Point d = to(device.body.p.x(), device.body.p.y());
  const cv::Point tip = to(device.body.p.x() + 1.8 * std::cos(yaw), device.body.p.y() + 1.8 * std::sin(yaw));
  cv::arrowedLine(img, d, tip, cv::Scalar(255, 255, 255), 2, cv::LINE_AA, 0, 0.5);
  cv::circle(img, d, 4, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);

  cv::rectangle(img, {0, 0}, {size_px - 1, size_px - 1}, cv::Scalar(200, 200, 200), 1);
  cv::putText(img, "N", {size_px - 16, 16}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(230, 230, 230),
    1, cv::LINE_AA);
  return img;
}

}  // namespace device_view
