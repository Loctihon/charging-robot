#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthHeatmap : public rclcpp::Node {
public:
    DepthHeatmap() : Node("depth_heatmap") {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/base_camera/base_camera_sensor/depth/image_raw", 10,
            std::bind(&DepthHeatmap::callback, this, std::placeholders::_1));
        pub_ = create_publisher<sensor_msgs::msg::Image>("/base_camera/depth_heatmap", 10);
        RCLCPP_INFO(this->get_logger(), "Depth heatmap C++ node started.");
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat depth;
        try {
            // Ép kiểu về Float32 để tính toán y hệt numpy
            depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Thay thế NaN và Inf bằng 0.0
        cv::patchNaNs(depth, 0.0);
        
        // Tạo mask cho các giá trị > 0
        cv::Mat valid_mask = depth > 0;
        
        double d_min, d_max;
        cv::minMaxLoc(depth, &d_min, &d_max, nullptr, nullptr, valid_mask);

        cv::Mat normalized = cv::Mat::zeros(depth.size(), CV_32FC1);
        if (d_max - d_min > 0) {
            normalized = (depth - d_min) / (d_max - d_min);
        }

        // Chuyển sang 8-bit (0-255)
        cv::Mat gray;
        normalized.convertTo(gray, CV_8UC1, 255.0);

        // Lật màu: gần = đỏ, xa = xanh (255 - gray)
        cv::Mat heatmap;
        cv::applyColorMap(gray, heatmap, cv::COLORMAP_JET);

        // Set các điểm ảnh không hợp lệ (depth <= 0) thành màu đen
        heatmap.setTo(cv::Scalar(0, 0, 0), depth <= 0);

        auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", heatmap).toImageMsg();
        pub_->publish(*out_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthHeatmap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}