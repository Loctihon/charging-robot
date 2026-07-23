/**
 * circle_detector_node.cpp
 *
 * Detects 7 circular holes on a charging socket face-plate from a binary mask,
 * then runs solvePnP to estimate the socket pose in camera frame.
 *
 * IMPORTANT:
 * - If detector finds extra lower/noise circles, this node keeps only the
 *   top 7 circles and ignores the rest.
 * - The valid CCS2 hole layout is grouped as U:2, M:3, L:2.
 *
 * Object-point layout in socket LOCAL frame, unit: metres:
 *
 * Origin: MC
 * +X right
 * +Y down on socket face
 * +Z into socket
 *
 * CAD coordinates:
 *   TL: (-8, 2.7, 801.2)    TR: ( 8, 2.7, 801.2)
 *   ML: (-16,4.2, 790.0)    MC: ( 0, 4.2, 790.0)    MR: (16,4.2,790.0)
 *   BL: (-8, 4.2, 776.1)    BR: ( 8, 4.2, 776.1)
 *
 * Converted local object points:
 *   x_local = Xcad / 1000
 *   y_down  = (790.0 - Zcad) / 1000
 *   z_local = 0
 *
 * Correspondence index order:
 *   [0] TL  [1] TR  [2] ML  [3] MC  [4] MR  [5] BL  [6] BR
 */

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>



class CircleDetectorNode : public rclcpp::Node {
public:
  CircleDetectorNode() : Node("circle_detector_node") {
    // Detection parameters
    this->declare_parameter("min_area", 5.0);
    this->declare_parameter("max_area", 5000.0);
    this->declare_parameter("min_circularity", 0.55);

    // Camera intrinsics
    this->declare_parameter("fx", 1513.7423);
    this->declare_parameter("fy", 1513.7423);
    this->declare_parameter("cx", 640.5);
    this->declare_parameter("cy", 360.5);

    // 3-D object points in socket LOCAL frame, metres
    // Index: [0] TL [1] TR [2] ML [3] MC [4] MR [5] BL [6] BR
    object_points_ = {
        cv::Point3f(-0.008f, -0.0112f, 0.000f), // [0] TL
        cv::Point3f( 0.008f, -0.0112f, 0.000f), // [1] TR

        cv::Point3f(-0.016f,  0.0000f, 0.000f), // [2] ML
        cv::Point3f( 0.000f,  0.0000f, 0.000f), // [3] MC
        cv::Point3f( 0.016f,  0.0000f, 0.000f), // [4] MR

        cv::Point3f(-0.008f,  0.0139f, 0.000f), // [5] BL
        cv::Point3f( 0.008f,  0.0139f, 0.000f), // [6] BR
    };

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ccs2_detector_node/raw_mask", 10,
        std::bind(&CircleDetectorNode::imageCb, this, std::placeholders::_1));

    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/circle_detector/debug_image", 10);

    center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/circle_detector/circle_center", 10);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/circle_detector/socket_pose", 10);

    tool_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/circle_detector/socket_pose_tool0", 10);

    RCLCPP_INFO(this->get_logger(),
                "CircleDetectorNode started. Top-7 filtering + fixed 2-3-2 grouping.");
  }

private:
  // Hardcoded static offset of camera optical frame relative to ur10_tool0.
  //
  // Axis remap (measured, not a small-angle rotation, but a full 90-deg-class
  // permutation with sign): tool_x = cam_z, tool_y = -cam_x, tool_z = cam_y.
  //   e.g. cam [-0.017, -0.1124, 0.7852] -> tool [0.7852, 0.017, -0.1124]
  // NOTE: this matrix has determinant -1 (a reflection, not a pure rotation)
  // because only the y-axis sign is flipped. That's fine for POSITION only.
  // The quaternion below (kAxisQ*) is NOT mathematically consistent with
  // this position matrix -- it still encodes the pure-rotation (det=+1)
  // version without the y flip. Orientation output is unused per current
  // requirements; if orientation is needed later, this must be revisited.
  static constexpr double kAxisR[3][3] = {
      {0.0,  0.0, 1.0},
      {-1.0, 0.0, 0.0},
      {0.0,  1.0, 0.0}
  };
  static constexpr double kAxisQx = 0.5;
  static constexpr double kAxisQy = 0.5;
  static constexpr double kAxisQz = 0.5;
  static constexpr double kAxisQw = 0.5;

  // Translation offset (meters) of camera origin relative to ur10_tool0,
  // expressed AFTER the axis remap above (i.e. in tool0-aligned axes).
  static constexpr double kCamOffsetX = 0.2312;
  static constexpr double kCamOffsetY = -0.0007;
  static constexpr double kCamOffsetZ = -0.0674;

  struct CircleInfo {
    int cx;
    int cy;
    int radius;
  };

  bool splitIntoGroups(
      const std::vector<CircleInfo> &sorted_by_y,
      std::vector<CircleInfo> &upper,
      std::vector<CircleInfo> &middle,
      std::vector<CircleInfo> &lower) const
  {
    const int N = static_cast<int>(sorted_by_y.size());
    if (N < 7) return false;

    // IMPORTANT:
    // Detector may find 9 circles:
    //   7 valid socket holes + 2 extra lower/noise holes.
    // We only keep the top 7 circles because image y smaller = physically upper.
    std::vector<CircleInfo> valid7(sorted_by_y.begin(), sorted_by_y.begin() + 7);

    upper.clear();
    middle.clear();
    lower.clear();

    // Fixed real layout: 2 - 3 - 2
    upper.insert(upper.end(),  valid7.begin(),     valid7.begin() + 2);
    middle.insert(middle.end(), valid7.begin() + 2, valid7.begin() + 5);
    lower.insert(lower.end(),  valid7.begin() + 5, valid7.begin() + 7);

    return true;
  }

  bool selectImagePoints(
      const std::vector<CircleInfo> &upper,
      const std::vector<CircleInfo> &middle,
      const std::vector<CircleInfo> &lower,
      std::vector<cv::Point2f> &img_pts,
      std::array<cv::Point2i, 7> &sel_pts) const
  {
    if (upper.size() != 2 || middle.size() != 3 || lower.size() != 2)
      return false;

    // Groups are already sorted left -> right before this function.
    const CircleInfo &tl = upper[0];
    const CircleInfo &tr = upper[1];

    const CircleInfo &ml = middle[0];
    const CircleInfo &mc = middle[1];
    const CircleInfo &mr = middle[2];

    const CircleInfo &bl = lower[0];
    const CircleInfo &br = lower[1];

    img_pts = {
        cv::Point2f((float)tl.cx, (float)tl.cy), // [0] TL
        cv::Point2f((float)tr.cx, (float)tr.cy), // [1] TR
        cv::Point2f((float)ml.cx, (float)ml.cy), // [2] ML
        cv::Point2f((float)mc.cx, (float)mc.cy), // [3] MC
        cv::Point2f((float)mr.cx, (float)mr.cy), // [4] MR
        cv::Point2f((float)bl.cx, (float)bl.cy), // [5] BL
        cv::Point2f((float)br.cx, (float)br.cy), // [6] BR
    };

    sel_pts = {{
        {tl.cx, tl.cy},
        {tr.cx, tr.cy},
        {ml.cx, ml.cy},
        {mc.cx, mc.cy},
        {mr.cx, mr.cy},
        {bl.cx, bl.cy},
        {br.cx, br.cy},
    }};

    return true;
  }

  static void rvecToQuat(const cv::Mat &rvec,
                         double &qx, double &qy, double &qz, double &qw) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    const double r00 = R.at<double>(0, 0);
    const double r01 = R.at<double>(0, 1);
    const double r02 = R.at<double>(0, 2);

    const double r10 = R.at<double>(1, 0);
    const double r11 = R.at<double>(1, 1);
    const double r12 = R.at<double>(1, 2);

    const double r20 = R.at<double>(2, 0);
    const double r21 = R.at<double>(2, 1);
    const double r22 = R.at<double>(2, 2);

    const double trace = r00 + r11 + r22;

    if (trace > 0.0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      qw = 0.25 / s;
      qx = (r21 - r12) * s;
      qy = (r02 - r20) * s;
      qz = (r10 - r01) * s;
    } else if (r00 > r11 && r00 > r22) {
      double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
      qw = (r21 - r12) / s;
      qx = 0.25 * s;
      qy = (r01 + r10) / s;
      qz = (r02 + r20) / s;
    } else if (r11 > r22) {
      double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
      qw = (r02 - r20) / s;
      qx = (r01 + r10) / s;
      qy = 0.25 * s;
      qz = (r12 + r21) / s;
    } else {
      double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
      qw = (r10 - r01) / s;
      qx = (r02 + r20) / s;
      qy = (r12 + r21) / s;
      qz = 0.25 * s;
    }
  }

  // Manual replacement for tf2::doTransform: composes the fixed camera-axis
  // remap (kAxisR / kAxis quaternion) with a translation offset to get the
  // pose expressed in ur10_tool0.
  //
  // p_tool = R_axis * p_cam + t_offset
  // q_tool = q_axis * q_cam
  static geometry_msgs::msg::PoseStamped applyStaticOffset(
      const geometry_msgs::msg::PoseStamped &pose_cam,
      double ox, double oy, double oz)
  {
    const double px = pose_cam.pose.position.x;
    const double py = pose_cam.pose.position.y;
    const double pz = pose_cam.pose.position.z;

    geometry_msgs::msg::PoseStamped out;
    out.header = pose_cam.header;
    out.header.frame_id = "ur10_tool0";

    out.pose.position.x = kAxisR[0][0] * px + kAxisR[0][1] * py + kAxisR[0][2] * pz + ox;
    out.pose.position.y = kAxisR[1][0] * px + kAxisR[1][1] * py + kAxisR[1][2] * pz + oy;
    out.pose.position.z = kAxisR[2][0] * px + kAxisR[2][1] * py + kAxisR[2][2] * pz + oz;

    const double q_cam_x = pose_cam.pose.orientation.x;
    const double q_cam_y = pose_cam.pose.orientation.y;
    const double q_cam_z = pose_cam.pose.orientation.z;
    const double q_cam_w = pose_cam.pose.orientation.w;

    // Hamilton product: q_tool = q_axis * q_cam
    out.pose.orientation.w = kAxisQw * q_cam_w - kAxisQx * q_cam_x - kAxisQy * q_cam_y - kAxisQz * q_cam_z;
    out.pose.orientation.x = kAxisQw * q_cam_x + kAxisQx * q_cam_w + kAxisQy * q_cam_z - kAxisQz * q_cam_y;
    out.pose.orientation.y = kAxisQw * q_cam_y - kAxisQx * q_cam_z + kAxisQy * q_cam_w + kAxisQz * q_cam_x;
    out.pose.orientation.z = kAxisQw * q_cam_z + kAxisQx * q_cam_y - kAxisQy * q_cam_x + kAxisQz * q_cam_w;

    return out;
  }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    const double min_area = this->get_parameter("min_area").as_double();
    const double max_area = this->get_parameter("max_area").as_double();
    const double min_circ = this->get_parameter("min_circularity").as_double();

    const double fx = this->get_parameter("fx").as_double();
    const double fy = this->get_parameter("fy").as_double();
    const double cx = this->get_parameter("cx").as_double();
    const double cy = this->get_parameter("cy").as_double();

    // Binarise + invert: holes become white blobs
    cv::Mat binary;
    cv::Mat inverted;
    cv::threshold(cv_ptr->image, binary, 127, 255, cv::THRESH_BINARY);
    cv::bitwise_not(binary, inverted);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(inverted, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    std::vector<CircleInfo> raw_circles;

    for (const auto &c : contours) {
      const double area = cv::contourArea(c);
      if (area < min_area || area > max_area) continue;

      const double perim = cv::arcLength(c, true);
      if (perim < 1e-5) continue;

      const double circularity = 4.0 * CV_PI * area / (perim * perim);
      if (circularity < min_circ) continue;

      const cv::Moments m = cv::moments(c);
      if (std::abs(m.m00) < 1e-5) continue;

      raw_circles.push_back({
          static_cast<int>(m.m10 / m.m00),
          static_cast<int>(m.m01 / m.m00),
          static_cast<int>(std::sqrt(area / CV_PI))
      });
    }

    // Merge duplicated detections
    std::vector<CircleInfo> circles;
    const float MERGE_DIST = 12.0f;

    for (const auto &c : raw_circles) {
      bool merged = false;

      for (auto &existing : circles) {
        const float dx = static_cast<float>(c.cx - existing.cx);
        const float dy = static_cast<float>(c.cy - existing.cy);

        if (std::sqrt(dx * dx + dy * dy) < MERGE_DIST) {
          if (c.radius > existing.radius) {
            existing = c;
          }
          merged = true;
          break;
        }
      }

      if (!merged) {
        circles.push_back(c);
      }
    }

    // Sort all detected circles by image Y, top -> bottom
    std::sort(circles.begin(), circles.end(),
              [](const CircleInfo &a, const CircleInfo &b) {
                return a.cy < b.cy;
              });

    RCLCPP_INFO(this->get_logger(),
                "Detected circles after merge = %zu", circles.size());

    std::vector<CircleInfo> upper_circles;
    std::vector<CircleInfo> mid_circles;
    std::vector<CircleInfo> lower_circles;

    const bool grouped = splitIntoGroups(
        circles, upper_circles, mid_circles, lower_circles);

    if (!grouped) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Too few circles: %zu < 7, skipping PnP.",
                           circles.size());
    } else {
      if (circles.size() > 7) {
        RCLCPP_INFO(this->get_logger(),
                    "Using top 7 circles only. Ignored %zu lower/noise circles.",
                    circles.size() - 7);
      }
    }

    // Sort each group left -> right
    auto by_x = [](const CircleInfo &a, const CircleInfo &b) {
      return a.cx < b.cx;
    };

    std::sort(upper_circles.begin(), upper_circles.end(), by_x);
    std::sort(mid_circles.begin(),   mid_circles.end(),   by_x);
    std::sort(lower_circles.begin(), lower_circles.end(), by_x);

    RCLCPP_INFO(this->get_logger(),
                "Groups -> U:%zu M:%zu L:%zu",
                upper_circles.size(),
                mid_circles.size(),
                lower_circles.size());

    // Debug image
    cv::Mat debug_img;
    cv::cvtColor(cv_ptr->image, debug_img, cv::COLOR_GRAY2BGR);

    const cv::Scalar COL_UPPER(0, 255, 0);
    const cv::Scalar COL_MID(0, 200, 255);
    const cv::Scalar COL_LOWER(255, 0, 255);
    const cv::Scalar COL_IGNORED(80, 80, 80);

    auto drawGroup = [&](const std::vector<CircleInfo> &grp,
                         const cv::Scalar &col) {
      double sx = 0.0;
      double sy = 0.0;

      for (const auto &c : grp) {
        cv::circle(debug_img, {c.cx, c.cy}, c.radius, col, 1);
        cv::circle(debug_img, {c.cx, c.cy}, 3, col, -1);
        sx += c.cx;
        sy += c.cy;
      }

      if (!grp.empty()) {
        cv::drawMarker(debug_img,
                       {static_cast<int>(sx / grp.size()),
                        static_cast<int>(sy / grp.size())},
                       col, cv::MARKER_CROSS, 16, 2);
      }
    };

    drawGroup(upper_circles, COL_UPPER);
    drawGroup(mid_circles,   COL_MID);
    drawGroup(lower_circles, COL_LOWER);

    // Draw ignored circles, if any
    if (circles.size() > 7) {
      for (size_t i = 7; i < circles.size(); ++i) {
        const auto &c = circles[i];
        cv::circle(debug_img, {c.cx, c.cy}, c.radius, COL_IGNORED, 1);
        cv::putText(debug_img, "IGN",
                    {c.cx + 5, c.cy - 5},
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    COL_IGNORED, 1);
      }
    }

    // Publish all raw centers
    for (const auto &c : circles) {
      geometry_msgs::msg::PointStamped pt;
      pt.header = msg->header;
      pt.point.x = c.cx;
      pt.point.y = c.cy;
      pt.point.z = c.radius;
      center_pub_->publish(pt);
    }

    std::vector<cv::Point2f> img_pts;
    std::array<cv::Point2i, 7> sel_pts;

    if (grouped &&
        selectImagePoints(upper_circles, mid_circles, lower_circles,
                          img_pts, sel_pts)) {
      const char *labels[] = {"TL", "TR", "ML", "MC", "MR", "BL", "BR"};

      const cv::Scalar label_colors[] = {
          cv::Scalar(0, 0, 255),
          cv::Scalar(0, 0, 255),

          cv::Scalar(0, 255, 255),
          cv::Scalar(0, 255, 255),
          cv::Scalar(0, 255, 255),

          cv::Scalar(255, 80, 0),
          cv::Scalar(255, 80, 0),
      };

      for (int i = 0; i < 7; ++i) {
        cv::circle(debug_img, sel_pts[i], 8, label_colors[i], 2);
        cv::putText(debug_img, labels[i],
                    {sel_pts[i].x + 6, sel_pts[i].y - 6},
                    cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    label_colors[i], 1);
      }

      const cv::Mat K =
          (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                     0, fy, cy,
                                     0,  0,  1);

      const cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);

      cv::Mat rvec;
      cv::Mat tvec;

      const bool ok = cv::solvePnP(
          object_points_,
          img_pts,
          K,
          D,
          rvec,
          tvec,
          false,
          cv::SOLVEPNP_ITERATIVE);

      if (ok) {
        cv::drawFrameAxes(debug_img, K, D, rvec, tvec, 0.05f, 2);

        std::vector<cv::Point2f> reproj;
        cv::projectPoints(object_points_, rvec, tvec, K, D, reproj);

        double err = 0.0;

        for (size_t i = 0; i < reproj.size(); ++i) {
          cv::circle(debug_img, reproj[i], 4, cv::Scalar(0, 165, 255), -1);

          const double dx = reproj[i].x - img_pts[i].x;
          const double dy = reproj[i].y - img_pts[i].y;
          err += std::sqrt(dx * dx + dy * dy);
        }

        err /= static_cast<double>(reproj.size());

        RCLCPP_INFO(this->get_logger(),
                    "PnP OK t=[%.4f, %.4f, %.4f] m reproj_err=%.2f px",
                    tvec.at<double>(0),
                    tvec.at<double>(1),
                    tvec.at<double>(2),
                    err);

        double qx, qy, qz, qw;
        rvecToQuat(rvec, qx, qy, qz, qw);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;

        pose.pose.position.x = tvec.at<double>(0);
        pose.pose.position.y = tvec.at<double>(1);
        pose.pose.position.z = tvec.at<double>(2);

        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;

        pose_pub_->publish(pose);

        // Camera-frame pose (pose) is published as-is via pose_pub_ above,
        // unchanged. Below we additionally publish the same pose re-expressed
        // in ur10_tool0 using the fixed axis remap + translation offset.
        geometry_msgs::msg::PoseStamped pose_tool =
            applyStaticOffset(pose, kCamOffsetX, kCamOffsetY, kCamOffsetZ);

        tool_pose_pub_->publish(pose_tool);

      } else {
        RCLCPP_WARN(this->get_logger(), "solvePnP failed.");
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Point selection failed. U:%zu M:%zu L:%zu",
                           upper_circles.size(),
                           mid_circles.size(),
                           lower_circles.size());
    }

    cv::putText(debug_img,
                "All:" + std::to_string(circles.size()) +
                " U:" + std::to_string(upper_circles.size()) +
                " M:" + std::to_string(mid_circles.size()) +
                " L:" + std::to_string(lower_circles.size()),
                {5, 18},
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(255, 200, 0),
                1);

    debug_pub_->publish(
        *cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg());
  }

private:
  std::vector<cv::Point3f> object_points_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_pub_;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}