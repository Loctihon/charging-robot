#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// Compile-time debug-pipeline switch.
// Set to true to display a 3×3 dashboard via cv::imshow() every frame.
// When false the node behaves exactly as before (zero overhead).
// ─────────────────────────────────────────────────────────────────────────────
constexpr bool SHOW_PIPELINE = true;

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/opencv.hpp>

#include "ccs2_socket_detector/msg/socket_detection.hpp"

namespace ccs2_socket_detector
{

// ─────────────────────────────────────────────────────────────────────────────
// Intermediate images captured during the gamma pipeline.
// Only populated when SHOW_PIPELINE == true.
// ─────────────────────────────────────────────────────────────────────────────
// 3×3 grid layout:
//   Row 0: RGB | Gray | CLAHE
//   Row 1: Gamma | Otsu Binary | Morph Cleanup
//   Row 2: Socket Shape | Final Detection | Circle Detector
struct PipelineImages
{
  cv::Mat gray;       // tile 1: BGR→Gray
  cv::Mat clahe;      // tile 2: CLAHE (or gray if disabled)
  cv::Mat gamma;      // tile 3: power-law transform
  cv::Mat otsu;       // tile 4: Otsu binary mask
  cv::Mat morph;      // tile 5: morphological cleanup
  cv::Mat socket;     // tile 6: socket shape (bitwise AND + morph)
  cv::Mat detection;  // tile 7: final BGR with bbox / centroid
};

class Ccs2DetectorNode : public rclcpp::Node
{
public:
  explicit Ccs2DetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Sync policy ───────────────────────────────────────────────────────────
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

  // ── Pipeline — MODE A: gamma (real hardware) ──────────────────────────────
  cv::Mat preprocessRGB(const cv::Mat & bgr, PipelineImages * pipe = nullptr);
  cv::Mat buildBinaryMaskGamma(const cv::Mat & preprocessed);

  // ── Pipeline — MODE B: HSV color filter (Gazebo / sim) ───────────────────
  cv::Mat buildBinaryMaskHSV(const cv::Mat & bgr);

  // ── Shared steps ──────────────────────────────────────────────────────────
  // scale_hint: 1.0 = close range, <1.0 = far (shrinks SE sizes)
  cv::Mat applyMorphCleanup(const cv::Mat & r_mask, double scale_hint = 1.0);
  cv::Mat extractSocketShape(
    const cv::Mat & binary_mask,
    const cv::Mat & binarized_median);
  bool    extractSocket(
    const cv::Mat & binary_mask,
    const cv::Mat & socket_shape,
    const cv::Mat & depth32f,
    ccs2_socket_detector::msg::SocketDetection & out,
    cv::RotatedRect & rotated_rect,
    cv::Rect & bbox);
  void    computeAngles(
    const cv::Mat & depth32f,
    const cv::Mat & mask,
    const cv::Rect & bbox,
    const cv::RotatedRect & rot,
    ccs2_socket_detector::msg::SocketDetection & out);

  // ── Adaptive helpers ──────────────────────────────────────────────────────
  // Returns a scale factor in [min_scale, 1.0] based on last known depth.
  // ref_depth is the close-range reference (full-size SE distance).
  double depthScale() const;
  // Round to nearest odd integer >= 3
  static int toOdd(int v) { return std::max(3, v % 2 == 0 ? v + 1 : v); }

  // ── Debug ─────────────────────────────────────────────────────────────────
  void publishDebug(
    const cv::Mat & bgr,
    const cv::Mat & mask,
    const cv::Mat & socket_shape,
    const ccs2_socket_detector::msg::SocketDetection & det,
    const cv::Rect & bbox,
    const std_msgs::msg::Header & header);

  // ── Pipeline visualization helpers (only active when SHOW_PIPELINE==true) ──
  // makeTile: resize + convert to BGR + draw title label on a 320×240 tile.
  static cv::Mat makeTile(const std::string & title, const cv::Mat & img,
                          cv::Size tile_size = cv::Size(320, 240));
  // showPipeline: assemble 9 tiles into a 3×3 dashboard and call imshow.
  // circle_img: latest frame from /circle_detector/debug_image (may be empty).
  static void showPipeline(const cv::Mat & rgb_bgr,
                           const PipelineImages & imgs,
                           const cv::Mat & circle_img);

  // ── ROS interfaces ────────────────────────────────────────────────────────
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<Synchronizer> sync_;

  // Subscriber for circle-detector debug image (displayed as last dashboard tile)
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr circle_det_sub_;

  rclcpp::Publisher<ccs2_socket_detector::msg::SocketDetection>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_mask_pub_;  // raw binary before morph fill

  // ── Parameters ────────────────────────────────────────────────────────────
  bool use_color_filter_;
  std::string image_encoding_;

  // --- Gamma pipeline params (MODE A) ---
  double gamma_;
  int    median_ksize_;
  int    morph_radius_;

  // --- CLAHE pre-enhancement (MODE A, helps at distance) ---
  bool   use_clahe_;
  double clahe_clip_;
  int    clahe_grid_;

  // --- HSV color filter params (MODE B) ---
  int hsv_h_low_;   int hsv_h_high_;
  int hsv_s_low_;   int hsv_s_high_;
  int hsv_v_low_;   int hsv_v_high_;

  // --- Shared params ---
  double socket_h_mm_;
  double socket_w_mm_;
  double min_area_;
  int    sync_queue_;

  // --- Distance-adaptive tuning ---
  // Morphological SE and filter sizes scale down as the socket moves farther.
  // ref_depth_m: the distance at which full-size parameters apply (close range).
  // min_scale:   floor on the scale factor (prevents SEs collapsing to 0).
  bool   adaptive_morph_;
  double ref_depth_m_;
  double min_scale_;

  // --- Aspect-ratio filter ---
  // CCS2 socket H≈135mm, W≈90mm → bbox height/width ≈ 1.5.
  // Set generous bounds to tolerate tilt and perspective.
  bool   use_aspect_filter_;
  double aspect_ratio_min_;   // bbox.height / bbox.width lower bound
  double aspect_ratio_max_;   // bbox.height / bbox.width upper bound

  // ── State ─────────────────────────────────────────────────────────────────
  float last_depth_;          // running estimate from previous frames (metres)

  // Latest image from /circle_detector/debug_image (protected by mutex).
  mutable std::mutex circle_img_mutex_;
  cv::Mat            last_circle_img_;  // empty until first message arrives
};

}  // namespace ccs2_socket_detector
