#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/opencv.hpp>

#include "ccs2_socket_detector/msg/socket_detection.hpp"

namespace ccs2_socket_detector
{

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
  cv::Mat preprocessRGB(const cv::Mat & bgr);
  cv::Mat buildBinaryMaskGamma(const cv::Mat & preprocessed);

  // ── Pipeline — MODE B: HSV color filter (Gazebo / sim) ───────────────────
  // Input is BGR (already converted from rgb8 correctly).
  // Returns binary mask isolating the gold/yellow socket region.
  cv::Mat buildBinaryMaskHSV(const cv::Mat & bgr);

  // ── Shared steps ──────────────────────────────────────────────────────────
  cv::Mat applyMorphCleanup(const cv::Mat & raw_mask);
  bool    extractSocket(
    const cv::Mat & mask,
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

  // ── Debug ─────────────────────────────────────────────────────────────────
  void publishDebug(
    const cv::Mat & bgr,
    const cv::Mat & mask,
    const ccs2_socket_detector::msg::SocketDetection & det,
    const cv::Rect & bbox,
    const std_msgs::msg::Header & header);

  // ── ROS interfaces ────────────────────────────────────────────────────────
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::Publisher<ccs2_socket_detector::msg::SocketDetection>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;

  // ── Parameters ────────────────────────────────────────────────────────────
  // true  → HSV color filter  (Gazebo / simulation)
  // false → gamma pipeline    (real ZED 2i hardware)
  bool use_color_filter_;

  // Input encoding from camera driver ("rgb8" or "bgr8")
  // Node always works internally in BGR — conversion is automatic.
  std::string image_encoding_;

  // --- Gamma pipeline params (MODE A) ---
  double gamma_;
  int    median_ksize_;
  int    morph_radius_;

  // --- HSV color filter params (MODE B) ---
  // Gold/yellow socket in Gazebo: H≈20-35, S≈100-255, V≈100-255
  // All tunable at runtime via ROS params.
  int hsv_h_low_;   int hsv_h_high_;
  int hsv_s_low_;   int hsv_s_high_;
  int hsv_v_low_;   int hsv_v_high_;

  // --- Shared params ---
  double socket_h_mm_;
  double socket_w_mm_;
  double min_area_;
  int    sync_queue_;
};

}  // namespace ccs2_socket_detector