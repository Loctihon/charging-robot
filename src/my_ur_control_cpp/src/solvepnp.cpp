
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

class CircleDetectorNode : public rclcpp::Node
{
public:
  CircleDetectorNode() : Node("circle_detector_node")
  {
    // --- Detection params ---
    this->declare_parameter("min_area",         5.0);
    this->declare_parameter("max_area",      5000.0);
    this->declare_parameter("min_circularity", 0.55);

    // --- Camera intrinsics ---
    this->declare_parameter("fx", 1513.7423);
    this->declare_parameter("fy", 1513.7423);
    this->declare_parameter("cx",  640.5);
    this->declare_parameter("cy",  360.5);

    // --- 3D object points in socket frame ---
    // Correspondence order (must match selectImagePoints output):
    //   [0] top-left   [1] top-right
    //   [2] mid-left   [3] mid-right
    //   [4] bot-left   [5] bot-right
    object_points_ = {
      {-0.08f,  0.027f, 0.0801f},
      { 0.08f,  0.027f, 0.0801f},
      {-0.016f, 0.042f, 0.079f},
      { 0.016f, 0.042f, 0.079f},
      {-0.08f,  0.042f, 0.0776f},
      { 0.08f,  0.042f, 0.0776f},
    };

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/ccs2_detector_node/raw_mask", 10,
      std::bind(&CircleDetectorNode::imageCb, this, std::placeholders::_1));

    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/circle_detector/debug_image", 10);

    center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/circle_detector/circle_center", 10);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/circle_detector/socket_pose", 10);

    RCLCPP_INFO(this->get_logger(), "CircleDetectorNode started.");
  }

private:
  struct CircleInfo { int cx, cy, radius; };

  // ── Select the 6 image points that match object_points_ ──────────────────
  //
  // BL and BR are the ONLY 2 circles in the lower group, so their
  // correspondence is unambiguous.  We use them as metric anchors:
  //
  //   pixel_scale  = |BR_px - BL_px| / |BR_3D.x - BL_3D.x|
  //                = |BR_px - BL_px| / 0.16  (pixels per metre)
  //
  // Then project the known 3D offset from BL/BR to each of TL/TR/ML/MR
  // into image space (front-facing approximation) and find the nearest
  // detected upper circle to each expected position.
  //
  // 3D offsets from anchor points (object frame):
  //   BL → TL : ΔX = 0,      ΔY = −0.015  (same column, 15 mm higher)
  //   BR → TR : ΔX = 0,      ΔY = −0.015
  //   BL → ML : ΔX = +0.064, ΔY =  0      (64 mm inward, same height)
  //   BR → MR : ΔX = −0.064, ΔY =  0
  //
  // ΔZ differences (≤ 2.5 mm) are ignored — they contribute < 1 px at 0.5 m.
  bool selectImagePoints(
    const std::vector<CircleInfo> & upper,
    const std::vector<CircleInfo> & lower,
    std::vector<cv::Point2f> & img_pts,
    std::array<cv::Point2i, 6> & sel_pts) const
  {
    if (upper.size() < 4 || lower.size() < 2) {
      return false;
    }

    // ── Anchors ───────────────────────────────────────────────────────────
    const cv::Point2f bl = {(float)lower.front().cx, (float)lower.front().cy};
    const cv::Point2f br = {(float)lower.back().cx,  (float)lower.back().cy};

    const float bl_br_px = br.x - bl.x;
    if (bl_br_px < 5.0f) { return false; }  // degenerate: DC pins too close

    // pixels per metre  (X direction; assume isotropic for small tilts)
    const float scale = bl_br_px / 0.16f;

    // ── Expected image positions ──────────────────────────────────────────
    // Image Y increases downward; 3D Y increases downward on socket face.
    // BL/ML share the same 3D Y (0.042), so same expected image Y.
    // TL/TR are at 3D Y = 0.027 → 0.015 m ABOVE BL/BR → dy_px < 0 (up).
    const float dy_top = -scale * 0.015f;   // negative = up in image
    const float dx_in  =  scale * 0.064f;   // inward offset BL→ML, BR→MR

    const cv::Point2f exp_tl = { bl.x,          bl.y + dy_top };
    const cv::Point2f exp_tr = { br.x,          br.y + dy_top };
    const cv::Point2f exp_ml = { bl.x + dx_in,  bl.y          };
    const cv::Point2f exp_mr = { br.x - dx_in,  br.y          };

    // ── Nearest upper circle to each expected position ────────────────────
    auto nearest = [&](cv::Point2f expected) -> const CircleInfo * {
      const CircleInfo * best = nullptr;
      float min_d2 = 1e9f;
      for (const auto & c : upper) {
        float dx = c.cx - expected.x, dy = c.cy - expected.y;
        float d2 = dx * dx + dy * dy;
        if (d2 < min_d2) { min_d2 = d2; best = &c; }
      }
      return best;
    };

    const CircleInfo * tl = nearest(exp_tl);
    const CircleInfo * tr = nearest(exp_tr);
    const CircleInfo * ml = nearest(exp_ml);
    const CircleInfo * mr = nearest(exp_mr);
    if (!tl || !tr || !ml || !mr) { return false; }

    img_pts = {
      { (float)tl->cx, (float)tl->cy },
      { (float)tr->cx, (float)tr->cy },
      { (float)ml->cx, (float)ml->cy },
      { (float)mr->cx, (float)mr->cy },
      { bl.x,          bl.y          },
      { br.x,          br.y          },
    };
    sel_pts = {{
      { tl->cx,          tl->cy          },
      { tr->cx,          tr->cy          },
      { ml->cx,          ml->cy          },
      { mr->cx,          mr->cy          },
      { (int)bl.x,       (int)bl.y       },
      { (int)br.x,       (int)br.y       },
    }};
    return true;
  }

  // ── rvec (Rodrigues) → quaternion (x, y, z, w) ──────────────────────────
  static void rvecToQuat(const cv::Mat & rvec,
                         double & qx, double & qy, double & qz, double & qw)
  {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    double r00 = R.at<double>(0,0), r01 = R.at<double>(0,1), r02 = R.at<double>(0,2);
    double r10 = R.at<double>(1,0), r11 = R.at<double>(1,1), r12 = R.at<double>(1,2);
    double r20 = R.at<double>(2,0), r21 = R.at<double>(2,1), r22 = R.at<double>(2,2);
    double trace = r00 + r11 + r22;
    if (trace > 0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      qw = 0.25 / s;
      qx = (r21 - r12) * s;
      qy = (r02 - r20) * s;
      qz = (r10 - r01) * s;
    } else if (r00 > r11 && r00 > r22) {
      double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
      qw = (r21 - r12) / s;  qx = 0.25 * s;
      qy = (r01 + r10) / s;  qz = (r02 + r20) / s;
    } else if (r11 > r22) {
      double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
      qw = (r02 - r20) / s;  qx = (r01 + r10) / s;
      qy = 0.25 * s;         qz = (r12 + r21) / s;
    } else {
      double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
      qw = (r10 - r01) / s;  qx = (r02 + r20) / s;
      qy = (r12 + r21) / s;  qz = 0.25 * s;
    }
  }

  // ── Main callback ─────────────────────────────────────────────────────────
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    double min_area = this->get_parameter("min_area").as_double();
    double max_area = this->get_parameter("max_area").as_double();
    double min_circ = this->get_parameter("min_circularity").as_double();

    // ── Binary + invert: holes become white blobs ─────────────────────────
    cv::Mat binary;
    cv::threshold(cv_ptr->image, binary, 127, 255, cv::THRESH_BINARY);
    cv::Mat inverted;
    cv::bitwise_not(binary, inverted);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(inverted, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // ── Pass 1: collect valid circles ─────────────────────────────────────
    std::vector<CircleInfo> circles;
    for (const auto & c : contours) {
      double area = cv::contourArea(c);
      if (area < min_area || area > max_area) continue;
      double perim = cv::arcLength(c, true);
      if (perim < 1e-5) continue;
      if (4.0 * CV_PI * area / (perim * perim) < min_circ) continue;
      cv::Moments m = cv::moments(c);
      if (std::abs(m.m00) < 1e-5) continue;
      circles.push_back({
        static_cast<int>(m.m10 / m.m00),
        static_cast<int>(m.m01 / m.m00),
        static_cast<int>(std::sqrt(area / CV_PI))
      });
    }

    // ── Pass 2: split into upper / lower by largest Y-gap ─────────────────
    std::sort(circles.begin(), circles.end(),
      [](const CircleInfo & a, const CircleInfo & b){ return a.cy < b.cy; });

    int split_idx = -1, max_gap = -1;
    for (size_t i = 1; i < circles.size(); ++i) {
      int gap = circles[i].cy - circles[i-1].cy;
      if (gap > max_gap) { max_gap = gap; split_idx = static_cast<int>(i) - 1; }
    }

    std::vector<CircleInfo> upper_circles, lower_circles;
    for (int i = 0; i < static_cast<int>(circles.size()); ++i) {
      if (split_idx < 0 || i <= split_idx) upper_circles.push_back(circles[i]);
      else                                  lower_circles.push_back(circles[i]);
    }

    // Sort each group by X (left → right) for point correspondence
    auto by_x = [](const CircleInfo & a, const CircleInfo & b){ return a.cx < b.cx; };
    std::sort(upper_circles.begin(), upper_circles.end(), by_x);
    std::sort(lower_circles.begin(), lower_circles.end(), by_x);

    // ── Debug image ───────────────────────────────────────────────────────
    cv::Mat debug_img;
    cv::cvtColor(cv_ptr->image, debug_img, cv::COLOR_GRAY2BGR);

    const cv::Scalar col_upper(  0, 255,   0);
    const cv::Scalar col_lower(255,   0, 255);

    double ucx = 0, ucy = 0, lcx = 0, lcy = 0;
    for (const auto & c : upper_circles) {
      cv::circle(debug_img, {c.cx, c.cy}, c.radius, col_upper, 1);
      cv::circle(debug_img, {c.cx, c.cy}, 3, cv::Scalar(255,255,0), -1);
      ucx += c.cx; ucy += c.cy;
    }
    for (const auto & c : lower_circles) {
      cv::circle(debug_img, {c.cx, c.cy}, c.radius, col_lower, 1);
      cv::circle(debug_img, {c.cx, c.cy}, 3, cv::Scalar(0,255,255), -1);
      lcx += c.cx; lcy += c.cy;
    }
    if (!upper_circles.empty()) {
      cv::drawMarker(debug_img,
        {(int)(ucx/upper_circles.size()), (int)(ucy/upper_circles.size())},
        col_upper, cv::MARKER_CROSS, 16, 2);
    }
    if (!lower_circles.empty()) {
      cv::drawMarker(debug_img,
        {(int)(lcx/lower_circles.size()), (int)(lcy/lower_circles.size())},
        col_lower, cv::MARKER_CROSS, 16, 2);
    }

    // Publish individual centers
    for (const auto & c : circles) {
      geometry_msgs::msg::PointStamped pt;
      pt.header = msg->header;
      pt.point.x = c.cx;  pt.point.y = c.cy;  pt.point.z = c.radius;
      center_pub_->publish(pt);
    }

    // ── SolvePnP ──────────────────────────────────────────────────────────
    std::vector<cv::Point2f> img_pts;
    std::array<cv::Point2i, 6> sel_pts;
    if (selectImagePoints(upper_circles, lower_circles, img_pts, sel_pts)) {

      const char * pt_labels[]    = {"TL","TR","ML","MR","BL","BR"};
      const cv::Scalar sel_colors[] = {
        {0,0,255},{0,0,255},        // TL/TR 
        {0,255,255},{0,255,255},    // ML/MR 
        {255,80,0},{255,80,0},      // BL/BR 
      };
      for (int i = 0; i < 6; ++i) {
        cv::circle(debug_img, sel_pts[i], 7, sel_colors[i], 2);
        cv::putText(debug_img, pt_labels[i],
          {sel_pts[i].x + 5, sel_pts[i].y - 5},
          cv::FONT_HERSHEY_SIMPLEX, 0.35, sel_colors[i], 1);
      }
      double fx = this->get_parameter("fx").as_double();
      double fy = this->get_parameter("fy").as_double();
      double cx = this->get_parameter("cx").as_double();
      double cy = this->get_parameter("cy").as_double();

      cv::Mat K = (cv::Mat_<double>(3,3) <<
        fx,  0, cx,
         0, fy, cy,
         0,  0,  1);
      cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);

      cv::Mat rvec, tvec;
      bool ok = cv::solvePnP(
        object_points_, img_pts, K, D, rvec, tvec,
        false, cv::SOLVEPNP_ITERATIVE);

      if (ok) {
        // Draw XYZ axes (red=X, green=Y, blue=Z), length 0.05 m
        cv::drawFrameAxes(debug_img, K, D, rvec, tvec, 0.05f, 2);

        // Reprojected points (orange dots) to check alignment
        std::vector<cv::Point2f> reproj;
        cv::projectPoints(object_points_, rvec, tvec, K, D, reproj);
        double err = 0;
        for (size_t i = 0; i < reproj.size(); ++i) {
          cv::circle(debug_img, reproj[i], 3, cv::Scalar(0,165,255), -1);  // orange
          double dx = reproj[i].x - img_pts[i].x;
          double dy = reproj[i].y - img_pts[i].y;
          err += std::sqrt(dx*dx + dy*dy);
        }
        err /= 6.0;

        RCLCPP_INFO(this->get_logger(),
          "PnP OK  t=[%.4f, %.4f, %.4f]m  reproj_err=%.2fpx",
          tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), err);

        // Publish PoseStamped
        double qx, qy, qz, qw;
        rvecToQuat(rvec, qx, qy, qz, qw);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x    = tvec.at<double>(0);
        pose.pose.position.y    = tvec.at<double>(1);
        pose.pose.position.z    = tvec.at<double>(2);
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;
        pose_pub_->publish(pose);
      } else {
        RCLCPP_WARN(this->get_logger(), "solvePnP failed.");
      }
    }

    // ── HUD ───────────────────────────────────────────────────────────────
    cv::putText(debug_img,
      "C:" + std::to_string(circles.size()) +
      " U:" + std::to_string(upper_circles.size()) +
      " L:" + std::to_string(lower_circles.size()),
      {5, 15}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,200,0), 1);

    debug_image_pub_->publish(
      *cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg());
  }

  // ── Members ───────────────────────────────────────────────────────────────
  std::vector<cv::Point3f>                                       object_points_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr       image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          debug_image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
