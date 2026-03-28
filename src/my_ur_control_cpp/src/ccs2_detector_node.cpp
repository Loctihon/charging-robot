#include "ccs2_socket_detector/ccs2_detector_node.hpp"

#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

namespace ccs2_socket_detector
{

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────
Ccs2DetectorNode::Ccs2DetectorNode(const rclcpp::NodeOptions & options)
: Node("ccs2_detector_node", options)
{
  // Mode switch: true = HSV color filter (Gazebo), false = gamma (real hw)
  this->declare_parameter("use_color_filter", true);
  this->declare_parameter("image_encoding",   std::string("rgb8"));

  // --- Gamma pipeline (MODE A: real hardware) ---
  this->declare_parameter("gamma",        0.001);
  this->declare_parameter("median_ksize", 9);
  this->declare_parameter("morph_radius", 18);

  // --- HSV filter (MODE B: Gazebo/sim) ---
  // Gold/yellow in OpenCV HSV (H:0-179, S:0-255, V:0-255)
  // Gold ≈ H:15-35, S:80-255, V:80-255
  this->declare_parameter("hsv_h_low",  8);
  this->declare_parameter("hsv_h_high", 29);
  this->declare_parameter("hsv_s_low",  223);
  this->declare_parameter("hsv_s_high", 255);
  this->declare_parameter("hsv_v_low",  21);
  this->declare_parameter("hsv_v_high", 64);

  // --- Shared ---
  this->declare_parameter("socket_h_mm", 135.0);
  this->declare_parameter("socket_w_mm", 90.0);
  this->declare_parameter("min_area",    500.0);
  this->declare_parameter("sync_queue",  10);

  // Fetch
  use_color_filter_ = this->get_parameter("use_color_filter").as_bool();
  image_encoding_   = this->get_parameter("image_encoding").as_string();

  gamma_        = this->get_parameter("gamma").as_double();
  median_ksize_ = this->get_parameter("median_ksize").as_int();
  morph_radius_ = this->get_parameter("morph_radius").as_int();
  if (median_ksize_ % 2 == 0) { median_ksize_++; }

  hsv_h_low_  = this->get_parameter("hsv_h_low").as_int();
  hsv_h_high_ = this->get_parameter("hsv_h_high").as_int();
  hsv_s_low_  = this->get_parameter("hsv_s_low").as_int();
  hsv_s_high_ = this->get_parameter("hsv_s_high").as_int();
  hsv_v_low_  = this->get_parameter("hsv_v_low").as_int();
  hsv_v_high_ = this->get_parameter("hsv_v_high").as_int();

  socket_h_mm_ = this->get_parameter("socket_h_mm").as_double();
  socket_w_mm_ = this->get_parameter("socket_w_mm").as_double();
  min_area_    = this->get_parameter("min_area").as_double();
  sync_queue_  = this->get_parameter("sync_queue").as_int();

  RCLCPP_INFO(this->get_logger(),
    "CCS2 detector — mode=%s  encoding=%s",
    use_color_filter_ ? "HSV_COLOR_FILTER" : "GAMMA_PIPELINE",
    image_encoding_.c_str());

  if (use_color_filter_) {
    RCLCPP_INFO(this->get_logger(),
      "HSV range — H:[%d,%d]  S:[%d,%d]  V:[%d,%d]",
      hsv_h_low_, hsv_h_high_,
      hsv_s_low_, hsv_s_high_,
      hsv_v_low_, hsv_v_high_);
  }

  // Subscribers
  rgb_sub_.subscribe(this, "/base_camera/base_camera_sensor/image_raw");
  depth_sub_.subscribe(this, "/base_camera/base_camera_sensor/depth/image_raw");
  sync_ = std::make_shared<Synchronizer>(
    SyncPolicy(sync_queue_), rgb_sub_, depth_sub_);
  sync_->registerCallback(&Ccs2DetectorNode::syncCallback, this);

  // Publishers
  det_pub_   = this->create_publisher<ccs2_socket_detector::msg::SocketDetection>(
    "~/socket_detection", 10);
  debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/debug_image", 10);
  mask_pub_  = this->create_publisher<sensor_msgs::msg::Image>("~/mask_image",  10);
}

// ─────────────────────────────────────────────────────────────────────────────
// Sync callback
// ─────────────────────────────────────────────────────────────────────────────
void Ccs2DetectorNode::syncCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{
  // ── RGB → BGR ─────────────────────────────────────────────────────────────
  // BUG FIX: Gazebo publishes "rgb8". Requesting "bgr8" from cv_bridge
  // forces the R↔B swap automatically — works regardless of source encoding.
  cv::Mat bgr;
  try {
    bgr = cv_bridge::toCvShare(rgb_msg, "bgr8")->image.clone();
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge RGB: %s", e.what());
    return;
  }

  // ── Depth → 32FC1 metres ──────────────────────────────────────────────────
  cv::Mat depth32f;
  try {
    auto dc = cv_bridge::toCvShare(depth_msg);
    if (dc->image.type() == CV_32FC1) {
      depth32f = dc->image.clone();
    } else if (dc->image.type() == CV_16UC1) {
      dc->image.convertTo(depth32f, CV_32FC1, 1.0 / 1000.0);  // mm → m
    } else {
      RCLCPP_ERROR(this->get_logger(),
        "Unexpected depth type: %s", depth_msg->encoding.c_str());
      return;
    }
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge depth: %s", e.what());
    return;
  }

  // ── Build raw mask (mode-dependent) ──────────────────────────────────────
  cv::Mat raw_mask;
  if (use_color_filter_) {
    raw_mask = buildBinaryMaskHSV(bgr);
  } else {
    cv::Mat pre = preprocessRGB(bgr);
    raw_mask = buildBinaryMaskGamma(pre);
  }

  cv::Mat mask = applyMorphCleanup(raw_mask);

  // ── Detect & compute ──────────────────────────────────────────────────────
  ccs2_socket_detector::msg::SocketDetection det;
  det.header = rgb_msg->header;

  cv::RotatedRect rot_rect;
  cv::Rect bbox;
  det.detected = extractSocket(mask, depth32f, det, rot_rect, bbox);

  if (det.detected) {
    computeAngles(depth32f, mask, bbox, rot_rect, det);
  }

  det_pub_->publish(det);

  if (debug_pub_->get_subscription_count() > 0 ||
      mask_pub_->get_subscription_count()  > 0)
  {
    publishDebug(bgr, mask, det, bbox, rgb_msg->header);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// MODE A — Gamma preprocess (real hardware)
// ─────────────────────────────────────────────────────────────────────────────
cv::Mat Ccs2DetectorNode::preprocessRGB(const cv::Mat & bgr)
{
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  cv::Mat se = cv::getStructuringElement(
    cv::MORPH_ELLIPSE,
    cv::Size(2 * morph_radius_ + 1, 2 * morph_radius_ + 1));
  cv::Mat dilated;
  cv::dilate(gray, dilated, se);

  // LUT for power-law gamma transform
  cv::Mat lut(1, 256, CV_8UC1);
  for (int i = 0; i < 256; ++i) {
    lut.at<uint8_t>(i) = cv::saturate_cast<uint8_t>(
      std::pow(i / 255.0, gamma_) * 255.0);
  }
  cv::Mat gamma_img;
  cv::LUT(dilated, lut, gamma_img);

  // Contrast stretch
  double mn, mx;
  cv::minMaxLoc(gamma_img, &mn, &mx);
  cv::Mat stretched;
  if (mx > mn) {
    gamma_img.convertTo(stretched, CV_8UC1,
      255.0 / (mx - mn), -mn * 255.0 / (mx - mn));
  } else {
    stretched = gamma_img.clone();
  }

  cv::Mat filtered;
  cv::medianBlur(stretched, filtered, median_ksize_);
  return filtered;
}

cv::Mat Ccs2DetectorNode::buildBinaryMaskGamma(const cv::Mat & preprocessed)
{
  cv::Mat binary;
  cv::threshold(preprocessed, binary, 0, 255,
    cv::THRESH_BINARY | cv::THRESH_OTSU);
  return binary;
}

// ─────────────────────────────────────────────────────────────────────────────
// MODE B — HSV color filter (Gazebo)
//
// Converts BGR → HSV, then inRange with tunable bounds.
// Gold/yellow in OpenCV HSV  (H:0-179, S:0-255, V:0-255):
//   H ≈ 15-35   S ≈ 80-255   V ≈ 80-255
//
// Tune by watching ~/mask_image in rqt_image_view:
//   - Socket not detected → lower hsv_s_low or hsv_v_low
//   - Background leaking in → narrow hsv_h range or raise hsv_s_low
// ─────────────────────────────────────────────────────────────────────────────
cv::Mat Ccs2DetectorNode::buildBinaryMaskHSV(const cv::Mat & bgr)
{
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(
    hsv,
    cv::Scalar(hsv_h_low_,  hsv_s_low_,  hsv_v_low_),
    cv::Scalar(hsv_h_high_, hsv_s_high_, hsv_v_high_),
    mask);

  return mask;
}

// ─────────────────────────────────────────────────────────────────────────────
// Shared morphological cleanup
// ─────────────────────────────────────────────────────────────────────────────
cv::Mat Ccs2DetectorNode::applyMorphCleanup(const cv::Mat & raw_mask)
{
  int r = std::max(3, morph_radius_ / 3);
  cv::Mat se_disk  = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1));
  cv::Mat se_cross = cv::getStructuringElement(
    cv::MORPH_CROSS,   cv::Size(2 * r + 1, 2 * r + 1));

  cv::Mat dilated;
  cv::dilate(raw_mask, dilated, se_cross, cv::Point(-1, -1), 2);

  cv::Mat closed;
  cv::morphologyEx(dilated, closed, cv::MORPH_CLOSE, se_disk,
    cv::Point(-1, -1), 2);

  // Hole fill: flood background → invert → OR back
  cv::Mat filled = closed.clone();
  cv::Mat flood_mask = cv::Mat::zeros(filled.rows + 2, filled.cols + 2, CV_8UC1);
  cv::floodFill(filled, flood_mask, cv::Point(0, 0), 255);
  cv::Mat filled_inv;
  cv::bitwise_not(filled, filled_inv);
  cv::bitwise_or(closed, filled_inv, filled);

  cv::Mat opened;
  cv::morphologyEx(filled, opened, cv::MORPH_OPEN, se_disk,
    cv::Point(-1, -1), 2);

  cv::Mat final_mask;
  cv::morphologyEx(opened, final_mask, cv::MORPH_CLOSE, se_disk,
    cv::Point(-1, -1), 2);

  return final_mask;
}

// ─────────────────────────────────────────────────────────────────────────────
// Extract socket — largest contour → centroid, bbox, depth
// ─────────────────────────────────────────────────────────────────────────────
bool Ccs2DetectorNode::extractSocket(
  const cv::Mat & mask,
  const cv::Mat & depth32f,
  ccs2_socket_detector::msg::SocketDetection & out,
  cv::RotatedRect & rotated_rect,
  cv::Rect & bbox)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "No contours.");
    return false;
  }

  size_t best = 0; double best_area = 0.0;
  for (size_t i = 0; i < contours.size(); ++i) {
    double a = cv::contourArea(contours[i]);
    if (a > best_area) { best_area = a; best = i; }
  }

  if (best_area < min_area_) {
    RCLCPP_DEBUG(this->get_logger(),
      "Contour area %.0f < min %.0f", best_area, min_area_);
    return false;
  }

  bbox = cv::boundingRect(contours[best]);

  // Angle via ellipse fit
  if (contours[best].size() >= 5) {
    rotated_rect = cv::fitEllipse(contours[best]);
    float a = rotated_rect.angle;
    if (a > 90.0f) { a -= 180.0f; }
    out.angle_xy = a;
  } else {
    rotated_rect = cv::minAreaRect(contours[best]);
    out.angle_xy = 0.0f;
  }

  // Centroid
  cv::Moments M = cv::moments(contours[best]);
  if (std::abs(M.m00) < 1e-6) { return false; }
  out.centroid_x = static_cast<float>(M.m10 / M.m00);
  out.centroid_y = static_cast<float>(M.m01 / M.m00);

  out.bbox_x      = static_cast<float>(bbox.x);
  out.bbox_y      = static_cast<float>(bbox.y);
  out.bbox_width  = static_cast<float>(bbox.width);
  out.bbox_height = static_cast<float>(bbox.height);

  // Average depth inside mask
  cv::Mat roi_mask  = mask(bbox);
  cv::Mat roi_depth = depth32f(bbox);
  std::vector<float> valid_depths;
  valid_depths.reserve(static_cast<size_t>(bbox.area()));
  for (int r = 0; r < roi_mask.rows; ++r) {
    for (int c = 0; c < roi_mask.cols; ++c) {
      if (roi_mask.at<uint8_t>(r, c) == 0) { continue; }
      float d = roi_depth.at<float>(r, c);
      if (std::isfinite(d) && d > 0.05f && d < 10.0f) {
        valid_depths.push_back(d);
      }
    }
  }
  if (!valid_depths.empty()) {
    double sum = std::accumulate(valid_depths.begin(), valid_depths.end(), 0.0);
    out.depth_m = static_cast<float>(sum / valid_depths.size());
  } else {
    out.depth_m = 0.0f;
  }

  RCLCPP_INFO(this->get_logger(),
    "Detected — centroid=(%.0f,%.0f)  bbox=[%d,%d %dx%d]  depth=%.3fm",
    out.centroid_x, out.centroid_y,
    bbox.x, bbox.y, bbox.width, bbox.height, out.depth_m);

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Compute tilt angles (paper eq. 11 & 12)
// ─────────────────────────────────────────────────────────────────────────────
void Ccs2DetectorNode::computeAngles(
  const cv::Mat & depth32f,
  const cv::Mat & mask,
  const cv::Rect & bbox,
  const cv::RotatedRect & /*rot*/,
  ccs2_socket_detector::msg::SocketDetection & out)
{
  if (bbox.width == 0 || bbox.height == 0) { return; }

  cv::Mat roi_mask  = mask(bbox);
  cv::Mat roi_depth = depth32f(bbox);

  auto row_avg = [&](int row) -> double {
    double s = 0.0; int n = 0;
    for (int c = 0; c < roi_mask.cols; ++c) {
      if (!roi_mask.at<uint8_t>(row, c)) { continue; }
      float d = roi_depth.at<float>(row, c);
      if (std::isfinite(d) && d > 0.05f && d < 10.0f) { s += d; ++n; }
    }
    return n > 0 ? s / n : std::numeric_limits<double>::quiet_NaN();
  };

  auto col_avg = [&](int col) -> double {
    double s = 0.0; int n = 0;
    for (int r = 0; r < roi_mask.rows; ++r) {
      if (!roi_mask.at<uint8_t>(r, col)) { continue; }
      float d = roi_depth.at<float>(r, col);
      if (std::isfinite(d) && d > 0.05f && d < 10.0f) { s += d; ++n; }
    }
    return n > 0 ? s / n : std::numeric_limits<double>::quiet_NaN();
  };

  // φ_YZ: (z_upper - z_lower) / H_real
  {
    int band = std::max(1, roi_mask.rows / 10);
    double su = 0.0, sl = 0.0; int cu = 0, cl = 0;
    for (int r = 0; r < band; ++r) {
      double d = row_avg(r);
      if (std::isfinite(d)) { su += d; ++cu; }
    }
    for (int r = roi_mask.rows - band; r < roi_mask.rows; ++r) {
      double d = row_avg(r);
      if (std::isfinite(d)) { sl += d; ++cl; }
    }
    if (cu > 0 && cl > 0) {
      double ratio = std::clamp(
        (su / cu - sl / cl) / (socket_h_mm_ / 1000.0), -1.0, 1.0);
      out.angle_yz = static_cast<float>(std::asin(ratio) * 180.0 / M_PI);
    }
  }

  // φ_XZ: (z_left - z_right) / W_real
  {
    int band = std::max(1, roi_mask.cols / 10);
    double sl = 0.0, sr = 0.0; int cl = 0, cr = 0;
    for (int c = 0; c < band; ++c) {
      double d = col_avg(c);
      if (std::isfinite(d)) { sl += d; ++cl; }
    }
    for (int c = roi_mask.cols - band; c < roi_mask.cols; ++c) {
      double d = col_avg(c);
      if (std::isfinite(d)) { sr += d; ++cr; }
    }
    if (cl > 0 && cr > 0) {
      double ratio = std::clamp(
        (sl / cl - sr / cr) / (socket_w_mm_ / 1000.0), -1.0, 1.0);
      out.angle_xz = static_cast<float>(std::asin(ratio) * 180.0 / M_PI);
    }
  }

  RCLCPP_INFO(this->get_logger(),
    "Angles — XY=%.2f°  YZ=%.2f°  XZ=%.2f°",
    out.angle_xy, out.angle_yz, out.angle_xz);
}

// ─────────────────────────────────────────────────────────────────────────────
// Debug publish
// ─────────────────────────────────────────────────────────────────────────────
void Ccs2DetectorNode::publishDebug(
  const cv::Mat & bgr,
  const cv::Mat & mask,
  const ccs2_socket_detector::msg::SocketDetection & det,
  const cv::Rect & bbox,
  const std_msgs::msg::Header & header)
{
  if (mask_pub_->get_subscription_count() > 0) {
    cv::Mat mc;
    cv::cvtColor(mask, mc, cv::COLOR_GRAY2BGR);
    mask_pub_->publish(*cv_bridge::CvImage(header, "bgr8", mc).toImageMsg());
  }

  if (debug_pub_->get_subscription_count() > 0) {
    cv::Mat viz = bgr.clone();

    if (det.detected) {
      // Semi-transparent overlay
      if (bbox.x >= 0 && bbox.y >= 0 &&
          bbox.x + bbox.width  <= viz.cols &&
          bbox.y + bbox.height <= viz.rows)
      {
        cv::Mat ov = viz.clone();
        cv::rectangle(ov, bbox, cv::Scalar(147, 20, 255), cv::FILLED);
        cv::addWeighted(ov, 0.3, viz, 0.7, 0, viz);
      }
      cv::rectangle(viz, bbox, cv::Scalar(0, 255, 0), 2);
      cv::drawMarker(viz,
        cv::Point(static_cast<int>(det.centroid_x),
                  static_cast<int>(det.centroid_y)),
        cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 24, 2);

      auto put = [&](const std::string & s, int line) {
        cv::Point p(10, 30 + line * 24);
        cv::putText(viz, s, p, cv::FONT_HERSHEY_SIMPLEX, 0.65,
          cv::Scalar(0,0,0), 3);
        cv::putText(viz, s, p, cv::FONT_HERSHEY_SIMPLEX, 0.65,
          cv::Scalar(255,255,255), 1);
      };
      char buf[128];
      snprintf(buf,sizeof(buf),"Mode: %s", use_color_filter_?"HSV":"GAMMA");
      put(buf,0);
      snprintf(buf,sizeof(buf),"Centroid: (%.0f, %.0f)",
        det.centroid_x, det.centroid_y);                    put(buf,1);
      snprintf(buf,sizeof(buf),"Depth: %.3f m",det.depth_m);put(buf,2);
      snprintf(buf,sizeof(buf),"XY: %.2f deg",det.angle_xy);put(buf,3);
      snprintf(buf,sizeof(buf),"YZ: %.2f deg",det.angle_yz);put(buf,4);
      snprintf(buf,sizeof(buf),"XZ: %.2f deg",det.angle_xz);put(buf,5);
    } else {
      cv::putText(viz, "No socket detected", cv::Point(10, 40),
        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0,0,255), 2);
    }

    debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", viz).toImageMsg());
  }
}

}  // namespace ccs2_socket_detector

// ─────────────────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<ccs2_socket_detector::Ccs2DetectorNode>());
  rclcpp::shutdown();
  return 0;
}