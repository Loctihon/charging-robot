#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <algorithm>
#include <cmath>

class PnPNode : public rclcpp::Node {
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
    PnPNode() : Node("pnp_node") {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/base_camera/base_camera_sensor/image_raw", 10,
            std::bind(&PnPNode::image_callback, this, std::placeholders::_1));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/object_pose", 10);
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/pnp_debug/image", 10);
        mask_pub_ = create_publisher<sensor_msgs::msg::Image>("/pnp_debug/mask", 10);
        tvec_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/pnp/tvec", 10);
        rvec_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/pnp/rvec", 10);
        
        double fx = 1280.0 / (2.0 * tan((60.0 * M_PI / 180.0) / 2.0));
        K_ = (cv::Mat_<double>(3, 3) << fx, 0, 640.0, 0, fx, 360.0, 0, 0, 1);
        dist_ = cv::Mat::zeros(4, 1, CV_64F);

        float hw = 0.045f, hh = 0.05875f;
        obj_pts_ = {
            cv::Point3f(-hw, hh, 0), cv::Point3f(hw, hh, 0),
            cv::Point3f(hw, -hh, 0), cv::Point3f(-hw, -hh, 0)
        };

        timer_ = create_wall_timer(std::chrono::milliseconds(33), std::bind(&PnPNode::process, this));
        RCLCPP_INFO(this->get_logger(), "PnP C++ Node (MULTI-SCALE SHAPE-BASED) started.");
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
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            latest_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void process() {
        if (latest_frame_.empty()) return;
        cv::Mat frame = latest_frame_.clone();

        // 1. CHUYỂN ẢNH SANG XÁM VÀ LẤY ĐƯỜNG VIỀN KHUNG HÌNH CHÍNH
        cv::Mat gray_frame, edges_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        cv::Canny(gray_frame, edges_frame, 20, 80);

        // 2. LOAD ẢNH MẪU PNG
        cv::Mat template_img = cv::imread("congsac2.png", cv::IMREAD_GRAYSCALE);
        if (template_img.empty()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "LỖI: Không tìm thấy ảnh congsac.png! Copy ra thư mục chạy lệnh ros2 run nha!");
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
        // 3. MULTI-SCALE MATCHING (Thuật toán cân mọi khoảng cách)
        double best_max_val = -1.0;
        cv::Point best_max_loc;
        double best_scale = 1.0;

        // Cho template phóng to/thu nhỏ từ 0.5 đến 2.0 lần (bước nhảy 0.1)
        for (double scale = 0.2; scale <= 2.0; scale += 0.1) {
            int new_w = (int)(template_img.cols * scale);
            int new_h = (int)(template_img.rows * scale);

            // Bỏ qua nếu template to hơn cả khung hình camera
            if (new_w >= edges_frame.cols || new_h >= edges_frame.rows) continue;

            cv::Mat resized_template, edges_template;
            // Resize ảnh gốc TRƯỚC, rồi mới trích xuất Canny để viền không bị bóp méo
            cv::resize(template_img, resized_template, cv::Size(new_w, new_h));
            cv::Canny(resized_template, edges_template, 20, 80);

            cv::Mat result_matrix;
            cv::matchTemplate(edges_frame, edges_template, result_matrix, cv::TM_CCOEFF_NORMED);

            double min_val, max_val;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(result_matrix, &min_val, &max_val, &min_loc, &max_loc);

            // Cập nhật nếu tìm thấy tỷ lệ khớp cao hơn
            if (max_val > best_max_val) {
                best_max_val = max_val;
                best_max_loc = max_loc;
                best_scale = scale;
            }
        }

        bool detected = false;

        // 4. NGƯỠNG TIN CẬY (Chọn > 0.25 là đẹp với Canny)
        if (best_max_val > 0.25) {
            // Tính toán lại kích thước khung hình sau khi Scale
            int final_w = (int)(template_img.cols * best_scale);
            int final_h = (int)(template_img.rows * best_scale);

            // LẤY 4 GÓC 2D
            cv::Point2f pt_TL(best_max_loc.x, best_max_loc.y);               
            cv::Point2f pt_TR(best_max_loc.x + final_w, best_max_loc.y);           
            cv::Point2f pt_BR(best_max_loc.x + final_w, best_max_loc.y + final_h);       
            cv::Point2f pt_BL(best_max_loc.x, best_max_loc.y + final_h);           

            std::vector<cv::Point2f> img_pts = {pt_TL, pt_TR, pt_BR, pt_BL};

            // Vẽ viền Debug
            std::vector<cv::Point> int_img_pts = {pt_TL, pt_TR, pt_BR, pt_BL};
            cv::polylines(frame, int_img_pts, true, cv::Scalar(0, 255, 255), 2);
            
            char text_info[50];
            sprintf(text_info, "Shape: %.2f | Scale: %.1f", best_max_val, best_scale);
            cv::putText(frame, text_info, pt_TL, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            // 5. BOM VÀO SOLVEPNP
            cv::Mat rvec, tvec;
            bool success = false;
            
            if (has_prev_extrinsic_) {
                rvec = prev_rvec_.clone();
                tvec = prev_tvec_.clone();
                success = cv::solvePnP(obj_pts_, img_pts, K_, dist_, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
            } else {
                success = cv::solvePnP(obj_pts_, img_pts, K_, dist_, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
            }

            // 6. CHECK SAI SỐ VÀ LÀM MƯỢT (SLERP)
            if (success) {
                std::vector<cv::Point2f> reproj_pts;
                cv::projectPoints(obj_pts_, rvec, tvec, K_, dist_, reproj_pts);
                double err = 0;
                for (size_t i = 0; i < img_pts.size(); ++i) {
                    err += cv::norm(reproj_pts[i] - img_pts[i]);
                }
                err /= img_pts.size();

                if (err <= 10.0) {
                    prev_rvec_ = rvec.clone();
                    prev_tvec_ = tvec.clone();
                    has_prev_extrinsic_ = true;

                    cv::Mat R;
                    cv::Rodrigues(rvec, R);
                    tf2::Matrix3x3 tf2_R(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                         R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                         R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
                    tf2::Quaternion rot_cur;
                    tf2_R.getRotation(rot_cur);

                    bool big_jump = false;
                    if (has_smooth_) {
                        double jump = cv::norm(tvec - tvec_smooth_);
                        if (jump > max_jump_) big_jump = true;
                    }

                    if (!has_smooth_ || big_jump) {
                        rot_smooth_ = rot_cur;
                        tvec_smooth_ = tvec.clone();
                        has_smooth_ = true;
                    } else {
                        rot_smooth_ = rot_smooth_.slerp(rot_cur, 1.0 - alpha_);
                        tvec_smooth_ = alpha_ * tvec_smooth_ + (1.0 - alpha_) * tvec;
                    }

                    publish_data(frame, rot_smooth_, tvec_smooth_);
                    detected = true;
                }
            }
        }

        if (!detected) {
            has_prev_extrinsic_ = false;
            cv::putText(frame, "NO DETECTION", cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        image_pub_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg());
        mask_pub_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", edges_frame).toImageMsg());
    }

    void publish_data(cv::Mat& frame, tf2::Quaternion q, cv::Mat tvec) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera";
        msg.pose.position.x = tvec.at<double>(0);
        msg.pose.position.y = tvec.at<double>(1);
        msg.pose.position.z = tvec.at<double>(2);
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        pose_pub_->publish(msg);

        tf2::Matrix3x3 R_smooth(q);
        cv::Mat R_mat = (cv::Mat_<double>(3,3) << R_smooth[0][0], R_smooth[0][1], R_smooth[0][2],
                                                  R_smooth[1][0], R_smooth[1][1], R_smooth[1][2],
                                                  R_smooth[2][0], R_smooth[2][1], R_smooth[2][2]);
        cv::Mat rvec_s;
        cv::Rodrigues(R_mat, rvec_s);

        geometry_msgs::msg::Vector3 t_msg, r_msg;
        t_msg.x = tvec.at<double>(0); t_msg.y = tvec.at<double>(1); t_msg.z = tvec.at<double>(2);
        r_msg.x = rvec_s.at<double>(0); r_msg.y = rvec_s.at<double>(1); r_msg.z = rvec_s.at<double>(2);
        tvec_pub_->publish(t_msg);
        rvec_pub_->publish(r_msg);

        std::vector<cv::Point3f> axis = {
            {0,0,0}, {0.05,0,0}, {0,0.05,0}, {0,0,0.05}
        };
        std::vector<cv::Point2f> imgpts;
        cv::projectPoints(axis, rvec_s, tvec, K_, dist_, imgpts);
        cv::line(frame, imgpts[0], imgpts[1], cv::Scalar(0,0,255), 3);
        cv::line(frame, imgpts[0], imgpts[2], cv::Scalar(0,255,0), 3);
        cv::line(frame, imgpts[0], imgpts[3], cv::Scalar(255,0,0), 3);

        char text[100];
        sprintf(text, "x=%.3f y=%.3f z=%.3f", t_msg.x, t_msg.y, t_msg.z);
        cv::putText(frame, text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_, mask_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr tvec_pub_, rvec_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    cv::Mat latest_frame_, K_, dist_;
    std::vector<cv::Point3f> obj_pts_;

    bool has_prev_extrinsic_ = false;
    cv::Mat prev_rvec_, prev_tvec_;
    
    bool has_smooth_ = false;
    tf2::Quaternion rot_smooth_;
    cv::Mat tvec_smooth_;
    double alpha_ = 0.3;
    double max_jump_ = 0.15;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PnPNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}