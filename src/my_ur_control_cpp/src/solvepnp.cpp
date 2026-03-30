
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

class CircleDetectorNode : public rclcpp::Node
{
public:
  CircleDetectorNode() : Node("circle_detector_node")
  {
    // --- tuneable params ---
    this->declare_parameter("min_area",         0.0);   // px^2  (filter single-pixel noise)
    this->declare_parameter("max_area",      5000.0);   // px^2  (generous upper bound)
    this->declare_parameter("min_circularity", 0.55);   // 0~1
    // RETR mode: 0=EXTERNAL, 1=LIST, 2=CCOMP, 3=TREE
    this->declare_parameter("retr_mode",          0);   // EXTERNAL is fine on isolated holes image

    // Subscribe to raw_mask: binary BEFORE morph cleanup fills the dark holes
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/ccs2_detector_node/raw_mask", 10,
      std::bind(&CircleDetectorNode::imageCb, this, std::placeholders::_1));

    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/circle_detector/debug_image", 10);

    center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/circle_detector/circle_center", 10);

    RCLCPP_INFO(this->get_logger(), "CircleDetectorNode started (hole-aware contour mode).");
  }

private:
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    double min_area  = this->get_parameter("min_area").as_double();
    double max_area  = this->get_parameter("max_area").as_double();
    double min_circ  = this->get_parameter("min_circularity").as_double();
    int    retr_mode = this->get_parameter("retr_mode").as_int();

    cv::Mat gray = cv_ptr->image.clone();

    // Clean binary: socket=255, holes=0, background=0
    cv::Mat binary;
    cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY);

    // Invert: socket=0 (invisible), holes=255, background=255
    // Background blob area >> max_area → filtered out automatically.
    // Socket body is 0 → not detected by findContours.
    // Dark holes inside socket → white blobs → detected as circles.
    cv::Mat inverted;
    cv::bitwise_not(binary, inverted);

    // Use RETR_LIST to get all contours without hierarchy overhead.
    // Background blob has enormous area and is removed by max_area filter.
    (void)retr_mode;  // kept as parameter for future use
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(inverted, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    RCLCPP_INFO(this->get_logger(), "Raw contours found: %zu", contours.size());

    // Debug image (BGR từ ảnh gốc)
    cv::Mat debug_img;
    cv::cvtColor(cv_ptr->image, debug_img, cv::COLOR_GRAY2BGR);

    int circle_count = 0;

    for (size_t i = 0; i < contours.size(); ++i) {
      double area = cv::contourArea(contours[i]);
      if (area < min_area || area > max_area) {
        // Background blob has huge area — only log small rejections to avoid spam
        if (area <= max_area * 2.0) {
          RCLCPP_INFO(this->get_logger(),
            "Contour %zu rejected: area=%.1f (range [%.1f, %.1f])", i, area, min_area, max_area);
        }
        continue;
      }

      double perimeter = cv::arcLength(contours[i], true);
      if (perimeter < 1e-5) continue;

      // Circularity = 4π·Area / P²
      double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
      if (circularity < min_circ) {
        RCLCPP_INFO(this->get_logger(),
          "Contour %zu rejected: circularity=%.2f < %.2f (area=%.1f)", i, circularity, min_circ, area);
        continue;
      }

      // Tâm qua image moments
      cv::Moments m = cv::moments(contours[i]);
      if (std::abs(m.m00) < 1e-5) continue;
      int cx = static_cast<int>(m.m10 / m.m00);
      int cy = static_cast<int>(m.m01 / m.m00);
      int radius = static_cast<int>(std::sqrt(area / CV_PI));

      // Vẽ debug
      cv::circle(debug_img, cv::Point(cx, cy), radius, cv::Scalar(0, 255, 0), 1);
      cv::circle(debug_img, cv::Point(cx, cy), 3,      cv::Scalar(0, 0, 255), -1);
      std::string label = "(" + std::to_string(cx) + "," + std::to_string(cy) + ")";
      cv::putText(debug_img, label, cv::Point(cx + 5, cy - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1);

      RCLCPP_INFO(this->get_logger(),
        "Circle[%d]: center=(%d,%d) r=%d area=%.1f circ=%.2f",
        circle_count, cx, cy, radius, area, circularity);

      // Publish tâm
      geometry_msgs::msg::PointStamped pt;
      pt.header = msg->header;
      pt.point.x = static_cast<double>(cx);
      pt.point.y = static_cast<double>(cy);
      pt.point.z = static_cast<double>(radius);
      center_pub_->publish(pt);

      circle_count++;
    }

    cv::putText(debug_img,
      "Circles: " + std::to_string(circle_count),
      cv::Point(5, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(255, 200, 0), 1);

    // RCLCPP_INFO(this->get_logger(), "Total circles: %d", circle_count);

    auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg();
    debug_image_pub_->publish(*debug_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr     debug_image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr center_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}   