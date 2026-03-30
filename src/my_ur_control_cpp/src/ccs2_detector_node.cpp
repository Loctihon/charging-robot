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
: Node("ccs2_detector_node", options),
  last_depth_(0.0f)
{
  // Mode switch: true = HSV color filter (Gazebo), false = gamma (real hw)
  this->declare_parameter("use_color_filter", false);
  this->declare_parameter("image_encoding",   std::string("rgb8"));

  // --- Gamma pipeline (MODE A: real hardware) ---
  this->declare_parameter("gamma",        0.001);
  this->declare_parameter("median_ksize", 9);
  this->declare_parameter("morph_radius", 12);

  // --- CLAHE pre-enhancement ---
  // Applying CLAHE before gamma lifts local contrast in dark regions,
  // which is the main reason detection degrades at distance.
  this->declare_parameter("use_clahe",  false);
  this->declare_parameter("clahe_clip",  2.0);
  this->declare_parameter("clahe_grid",  8);

  // --- HSV filter (MODE B: Gazebo/sim) ---
  this->declare_parameter("hsv_h_low",  8);
  this->declare_parameter("hsv_h_high", 29);
  this->declare_parameter("hsv_s_low",  150);
  this->declare_parameter("hsv_s_high", 255);
  this->declare_parameter("hsv_v_low",  15);
  this->declare_parameter("hsv_v_high", 80);

  // --- Shared ---
  this->declare_parameter("socket_h_mm", 135.0);
  this->declare_parameter("socket_w_mm", 90.0);
  this->declare_parameter("min_area",    500.0);
  this->declare_parameter("sync_queue",  10);

  // --- Distance-adaptive morphology ---
  // The paper says: "SEs are size-dependent matrices … can be simply tuned
  // depending on the size of the objects in the image, where the size of the
  // shapes in the image depends on … the range of the depth sensor from the
  // object itself."  We automate this using the last known depth.
  this->declare_parameter("adaptive_morph", true);
  this->declare_parameter("ref_depth_m",    0.5);   // full-size SE distance
  this->declare_parameter("min_scale",      0.20);  // minimum SE scale

  // --- Aspect-ratio filter ---
  // CCS2 socket: H≈135mm, W≈90mm → height/width ≈ 1.5 in a frontal view.
  // Filtering by this ratio rejects large rectangular false positives (walls,
  // frames) that have the wrong proportions.
  this->declare_parameter("use_aspect_filter",  true);
  this->declare_parameter("aspect_ratio_min",   0.55);  // generous for tilt
  this->declare_parameter("aspect_ratio_max",   2.80);

  // Fetch all
  use_color_filter_ = this->get_parameter("use_color_filter").as_bool();
  image_encoding_   = this->get_parameter("image_encoding").as_string();

  gamma_        = this->get_parameter("gamma").as_double();
  median_ksize_ = this->get_parameter("median_ksize").as_int();
  morph_radius_ = this->get_parameter("morph_radius").as_int();
  if (median_ksize_ % 2 == 0) { median_ksize_++; }

  use_clahe_   = this->get_parameter("use_clahe").as_bool();
  clahe_clip_  = this->get_parameter("clahe_clip").as_double();
  clahe_grid_  = this->get_parameter("clahe_grid").as_int();

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

  adaptive_morph_ = this->get_parameter("adaptive_morph").as_bool();
  ref_depth_m_    = this->get_parameter("ref_depth_m").as_double();
  min_scale_      = this->get_parameter("min_scale").as_double();

  use_aspect_filter_  = this->get_parameter("use_aspect_filter").as_bool();
  aspect_ratio_min_   = this->get_parameter("aspect_ratio_min").as_double();
  aspect_ratio_max_   = this->get_parameter("aspect_ratio_max").as_double();

  RCLCPP_INFO(this->get_logger(),
    "CCS2 detector — mode=%s  encoding=%s  clahe=%s  adaptive_morph=%s  aspect_filter=%s",
    use_color_filter_ ? "HSV_COLOR_FILTER" : "GAMMA_PIPELINE",
    image_encoding_.c_str(),
    use_clahe_ ? "ON" : "OFF",
    adaptive_morph_ ? "ON" : "OFF",
    use_aspect_filter_ ? "ON" : "OFF");

  // Subscribers
  rgb_sub_.subscribe(this, "/base_camera/base_camera_sensor/image_raw");
  depth_sub_.subscribe(this, "/base_camera/base_camera_sensor/depth/image_raw");
  sync_ = std::make_shared<Synchronizer>(
    SyncPolicy(sync_queue_), rgb_sub_, depth_sub_);
  sync_->registerCallback(&Ccs2DetectorNode::syncCallback, this);

  // Publishers
  det_pub_      = this->create_publisher<ccs2_socket_detector::msg::SocketDetection>(
    "~/socket_detection", 10);
  debug_pub_    = this->create_publisher<sensor_msgs::msg::Image>("~/debug_image", 10);
  mask_pub_     = this->create_publisher<sensor_msgs::msg::Image>("~/mask_image",  10);
  raw_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/raw_mask",    10);
}

// ─────────────────────────────────────────────────────────────────────────────
// depthScale
//
// Returns a scale factor in [min_scale_, 1.0] that shrinks morphological
// parameters as the socket moves farther away.
//
//   scale = clamp(ref_depth / last_depth, min_scale, 1.0)
//
// At ref_depth (close range): scale = 1.0 → full SE sizes.
// At 2×ref_depth            : scale = 0.5 → half SE sizes.
// At 4×ref_depth            : scale = 0.25 → clamped to min_scale.
// ─────────────────────────────────────────────────────────────────────────────
double Ccs2DetectorNode::depthScale() const
{
  if (!adaptive_morph_ || last_depth_ < 0.05f) {
    return 1.0;
  }
  double scale = ref_depth_m_ / static_cast<double>(last_depth_);
  return std::clamp(scale, min_scale_, 1.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Sync callback
// ─────────────────────────────────────────────────────────────────────────────
void Ccs2DetectorNode::syncCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{
  // ── RGB → BGR ─────────────────────────────────────────────────────────────
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
      dc->image.convertTo(depth32f, CV_32FC1, 1.0 / 1000.0);
    } else {
      RCLCPP_ERROR(this->get_logger(),
        "Unexpected depth type: %s", depth_msg->encoding.c_str());
      return;
    }
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge depth: %s", e.what());
    return;
  }

  // ── Compute depth-based scale for this frame ──────────────────────────────
  // Use the previous frame's depth to set SE sizes for the current frame.
  // On the very first frame last_depth_==0 → scale=1.0 (full parameters).
  double scale = depthScale();

  // ── Build raw mask (mode-dependent) ──────────────────────────────────────
  cv::Mat raw_mask;
  cv::Mat binarized_median;

  if (use_color_filter_) {
    raw_mask = buildBinaryMaskHSV(bgr);
    binarized_median = raw_mask.clone();
  } else {
    cv::Mat pre = preprocessRGB(bgr);
    raw_mask         = buildBinaryMaskGamma(pre);
    binarized_median = raw_mask.clone();
  }

  // ── Publish raw binary mask for circle detection (before morph fills holes) ─
  if (raw_mask_pub_->get_subscription_count() > 0) {
    cv::Mat raw_bgr;
    cv::cvtColor(binarized_median, raw_bgr, cv::COLOR_GRAY2BGR);
    raw_mask_pub_->publish(
      *cv_bridge::CvImage(rgb_msg->header, "bgr8", raw_bgr).toImageMsg());
  }

  // ── Morphological cleanup — pass scale so SEs shrink when far ─────────────
  cv::Mat mask = applyMorphCleanup(raw_mask, scale);

  // ── Paper step: logical mult → socket shape ────────────────────────────────
  cv::Mat socket_shape = extractSocketShape(mask, binarized_median);

  // ── Detect & compute ──────────────────────────────────────────────────────
  ccs2_socket_detector::msg::SocketDetection det;
  det.header = rgb_msg->header;

  cv::RotatedRect rot_rect;
  cv::Rect bbox;
  det.detected = extractSocket(mask, socket_shape, depth32f, det, rot_rect, bbox);

  if (det.detected) {
    computeAngles(depth32f, socket_shape, bbox, rot_rect, det);
    // Update last_depth_ for the next frame's SE scaling.
    if (det.depth_m > 0.05f) {
      // Exponential moving average so a single bad depth reading doesn't
      // immediately destroy the scale estimate.
      last_depth_ = 0.8f * last_depth_ + 0.2f * det.depth_m;
      if (last_depth_ < 0.05f) { last_depth_ = det.depth_m; }
    }
  }

  det_pub_->publish(det);

  if (debug_pub_->get_subscription_count() > 0 ||
      mask_pub_->get_subscription_count()  > 0)
  {
    publishDebug(bgr, mask, socket_shape, det, bbox, rgb_msg->header);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// MODE A — Gamma preprocess (real hardware)
//
// Changes vs. original:
//   1. Optional CLAHE pass before gamma — significantly improves local contrast
//      in the dark socket region when the camera is far from the socket.
//   2. Adaptive dilation SE and median filter size — both scale down via
//      depthScale() so features are not over-smoothed at range.
// ─────────────────────────────────────────────────────────────────────────────
cv::Mat Ccs2DetectorNode::preprocessRGB(const cv::Mat & bgr)
{
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  // ── 1. CLAHE (optional) ───────────────────────────────────────────────────
  // Contrast-Limited Adaptive Histogram Equalization boosts local contrast in
  // the dark socket area that is otherwise crushed by the gamma transform when
  // the socket is small (far away).  The paper's gamma alone works well at
  // close range but loses the socket texture at distance.
  if (use_clahe_) {
    auto clahe = cv::createCLAHE(clahe_clip_,
      cv::Size(clahe_grid_, clahe_grid_));
    clahe->apply(gray, gray);
  }

  // ── 2. Grayscale dilation — adaptive radius ───────────────────────────────
  // The paper uses a disk-shaped SE here.  Scale the radius with depth so a
  // distant (small) socket is not destroyed by an over-large SE.
  double scale = depthScale();
  int eff_radius = std::max(1, static_cast<int>(morph_radius_ * scale));
  cv::Mat se = cv::getStructuringElement(
    cv::MORPH_ELLIPSE,
    cv::Size(2 * eff_radius + 1, 2 * eff_radius + 1));
  cv::Mat dilated;
  cv::dilate(gray, dilated, se);

  // ── 3. Gamma (power-law intensity transform) ──────────────────────────────
  cv::Mat lut(1, 256, CV_8UC1);
  for (int i = 0; i < 256; ++i) {
    lut.at<uint8_t>(i) = cv::saturate_cast<uint8_t>(
      std::pow(i / 255.0, gamma_) * 255.0);
  }
  cv::Mat gamma_img;
  cv::LUT(dilated, lut, gamma_img);

  // ── 4. Contrast stretch ───────────────────────────────────────────────────
  double mn, mx;
  cv::minMaxLoc(gamma_img, &mn, &mx);
  cv::Mat stretched;
  if (mx > mn) {
    gamma_img.convertTo(stretched, CV_8UC1,
      255.0 / (mx - mn), -mn * 255.0 / (mx - mn));
  } else {
    stretched = gamma_img.clone();
  }

  // ── 5. Median filter — adaptive kernel size ───────────────────────────────
  // At close range a 9×9 kernel works well.  At distance the socket occupies
  // fewer pixels; a large kernel merges it into the background.
  int eff_median = toOdd(static_cast<int>(median_ksize_ * scale));
  cv::Mat filtered;
  cv::medianBlur(stretched, filtered, eff_median);
  return filtered;
}

cv::Mat Ccs2DetectorNode::buildBinaryMaskGamma(const cv::Mat & preprocessed)
{
  cv::Mat binary;
  cv::threshold(preprocessed, binary, 0, 255,
    cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  return binary;
}

// ─────────────────────────────────────────────────────────────────────────────
// MODE B — HSV color filter (Gazebo)
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
//
// scale_hint scales all SE radii proportionally.  When the socket is far
// (small in the image), smaller SEs preserve detail; when close, larger SEs
// fill gaps and noise more aggressively.
// ─────────────────────────────────────────────────────────────────────────────
cv::Mat Ccs2DetectorNode::applyMorphCleanup(const cv::Mat & raw_mask,
                                             double scale_hint)
{
  int base_r = morph_radius_ / 3;
  int r = std::max(1, static_cast<int>(base_r * scale_hint));

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
// Paper step (Fig. 8a→8b): logical mult → socket shape, small morph cleanup
// ─────────────────────────────────────────────────────────────────────────────
cv::Mat Ccs2DetectorNode::extractSocketShape(
  const cv::Mat & binary_mask,
  const cv::Mat & binarized_median)
{
  cv::Mat shape;
  cv::bitwise_and(binary_mask, binarized_median, shape);

  double scale = depthScale();
  int r = std::max(1, static_cast<int>((morph_radius_ / 6) * scale));
  cv::Mat se_cross = cv::getStructuringElement(
    cv::MORPH_CROSS,   cv::Size(2 * r + 1, 2 * r + 1));
  cv::Mat se_disk  = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1));

  cv::Mat step;
  cv::dilate(shape, step, se_cross, cv::Point(-1, -1), 1);
  cv::morphologyEx(step, step, cv::MORPH_CLOSE, se_disk, cv::Point(-1, -1), 1);
  cv::morphologyEx(step, step, cv::MORPH_OPEN,  se_disk, cv::Point(-1, -1), 1);
  cv::erode(step, step, se_cross, cv::Point(-1, -1), 1);

  return step;
}

// ─────────────────────────────────────────────────────────────────────────────
// Extract socket — contour selection, centroid, bbox, depth
//
// Key improvements over the original:
//
// 1. Adaptive min_area — at distance the socket occupies far fewer pixels.
//    Area scales as (ref_depth/depth)² so we lower the threshold when far.
//
// 2. Aspect-ratio filter — the CCS2 socket has height ≈ 1.5× its width.
//    Large flat false positives (walls, frames) have the wrong ratio and are
//    rejected before they can outcompete the real socket by area alone.
//
// 3. Scoring — candidates are ranked by a combined score:
//      score = area × aspect_score
//    where aspect_score = 1.0 when the ratio exactly matches the expected
//    CCS2 ratio (socket_h_mm / socket_w_mm) and falls off smoothly away.
//    This means a smaller-area contour with the right shape can beat a
//    larger-area blob with the wrong shape.
// ─────────────────────────────────────────────────────────────────────────────
bool Ccs2DetectorNode::extractSocket(
  const cv::Mat & binary_mask,
  const cv::Mat & socket_shape,
  const cv::Mat & depth32f,
  ccs2_socket_detector::msg::SocketDetection & out,
  cv::RotatedRect & rotated_rect,
  cv::Rect & bbox)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "No contours.");
    return false;
  }

  // Adaptive minimum area: scales as square of depth ratio because pixel count
  // per unit area shrinks quadratically with distance.
  double scale = depthScale();
  double eff_min_area = std::max(50.0, min_area_ * scale * scale);

  // Expected aspect ratio for the CCS2 socket (height / width in pixels).
  // In a frontal view this equals socket_h_mm / socket_w_mm ≈ 1.5.
  // Under tilt the ratio shifts, so we only soft-score, not hard-reject.
  double expected_ratio = socket_h_mm_ / socket_w_mm_;

  // ── Score each candidate ──────────────────────────────────────────────────
  int    best = -1;
  double best_score = -1.0;

  for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area < eff_min_area) { continue; }

    cv::Rect bb = cv::boundingRect(contours[i]);
    if (bb.width == 0 || bb.height == 0) { continue; }

    double ratio = static_cast<double>(bb.height) / static_cast<double>(bb.width);

    // Hard aspect-ratio gate (generous bounds, catches extreme false positives)
    if (use_aspect_filter_) {
      if (ratio < aspect_ratio_min_ || ratio > aspect_ratio_max_) {
        RCLCPP_DEBUG(this->get_logger(),
          "Contour %zu rejected: ratio=%.2f out of [%.2f, %.2f]",
          i, ratio, aspect_ratio_min_, aspect_ratio_max_);
        continue;
      }
    }

    // Soft aspect-ratio score: Gaussian centred on expected_ratio.
    // σ = 0.5 keeps the score > 0.6 within ±0.5 of the ideal ratio.
    double diff  = (ratio - expected_ratio) / 0.5;
    double ascore = std::exp(-0.5 * diff * diff);

    double score = area * ascore;
    if (score > best_score) {
      best_score = score;
      best = static_cast<int>(i);
    }
  }

  if (best < 0) {
    RCLCPP_DEBUG(this->get_logger(),
      "No contour passed aspect+area filter (min_area=%.0f).", eff_min_area);
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

  // ── Depth extraction (paper Fig.10) ──────────────────────────────────────
  const cv::Mat & depth_mask = socket_shape.empty() ? binary_mask : socket_shape;
  cv::Mat roi_mask  = depth_mask(bbox);
  cv::Mat roi_depth = depth32f(bbox);

  std::vector<float> valid_depths;
  valid_depths.reserve(static_cast<size_t>(bbox.area()));
  for (int row = 0; row < roi_mask.rows; ++row) {
    for (int col = 0; col < roi_mask.cols; ++col) {
      if (roi_mask.at<uint8_t>(row, col) == 0) { continue; }
      float d = roi_depth.at<float>(row, col);
      if (std::isfinite(d) && d > 0.05f && d < 10.0f) {
        valid_depths.push_back(d);
      }
    }
  }
  if (!valid_depths.empty()) {
    // Use median depth (more robust to NaN holes in the depth map at distance)
    std::nth_element(valid_depths.begin(),
      valid_depths.begin() + valid_depths.size() / 2,
      valid_depths.end());
    out.depth_m = valid_depths[valid_depths.size() / 2];
  } else {
    out.depth_m = 0.0f;
  }

  double ratio_used = static_cast<double>(bbox.height) / static_cast<double>(bbox.width);
  RCLCPP_INFO(this->get_logger(),
    "Detected — centroid=(%.0f,%.0f)  bbox=[%d,%d %dx%d]  H/W=%.2f  depth=%.3fm  scale=%.2f",
    out.centroid_x, out.centroid_y,
    bbox.x, bbox.y, bbox.width, bbox.height,
    ratio_used, out.depth_m, scale);

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

  // φ_YZ (paper eq. 11)
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

  // φ_XZ (paper eq. 12)
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
  const cv::Mat & socket_shape,
  const ccs2_socket_detector::msg::SocketDetection & det,
  const cv::Rect & bbox,
  const std_msgs::msg::Header & header)
{
  if (!socket_shape.empty()) {
    cv::Mat sc;
    cv::cvtColor(socket_shape, sc, cv::COLOR_GRAY2BGR);
    mask_pub_->publish(*cv_bridge::CvImage(header, "bgr8", sc).toImageMsg());
  }

  if (debug_pub_->get_subscription_count() > 0) {
    cv::Mat viz = bgr.clone();

    if (det.detected) {
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
      snprintf(buf, sizeof(buf), "Mode: %s", use_color_filter_ ? "HSV" : "GAMMA");
      put(buf, 0);
      snprintf(buf, sizeof(buf), "Centroid: (%.0f, %.0f)",
        det.centroid_x, det.centroid_y);                      put(buf, 1);
      snprintf(buf, sizeof(buf), "Depth: %.3f m", det.depth_m); put(buf, 2);
      snprintf(buf, sizeof(buf), "XY: %.2f deg", det.angle_xy); put(buf, 3);
      snprintf(buf, sizeof(buf), "YZ: %.2f deg", det.angle_yz); put(buf, 4);
      snprintf(buf, sizeof(buf), "XZ: %.2f deg", det.angle_xz); put(buf, 5);
      snprintf(buf, sizeof(buf), "Scale: %.2f", depthScale());   put(buf, 6);
    } else {
      cv::putText(viz, "No socket detected", cv::Point(10, 40),
        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
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
