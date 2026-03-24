#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <numeric>

class PnPNode : public rclcpp::Node {
public:
    PnPNode() : Node("pnp_node") {
        // Subscribers & Publishers
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/base_camera/base_camera_sensor/image_raw", 10,
            std::bind(&PnPNode::image_callback, this, std::placeholders::_1));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/object_pose", 10);
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/pnp_debug/image", 10);
        mask_pub_ = create_publisher<sensor_msgs::msg::Image>("/pnp_debug/mask", 10);
        tvec_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/pnp/tvec", 10);
        rvec_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/pnp/rvec", 10);
        
        // Camera Params (1280x720, HFOV 60)
        double fx = 1280.0 / (2.0 * tan((60.0 * M_PI / 180.0) / 2.0));
        K_ = (cv::Mat_<double>(3, 3) << fx, 0, 640.0, 0, fx, 360.0, 0, 0, 1);
        dist_ = cv::Mat::zeros(4, 1, CV_64F);

        // Object points (90mm x 117.5mm)
        float hw = 0.045f, hh = 0.05875f;
        obj_pts_ = {
            cv::Point3f(-hw, hh, 0),  // TL
            cv::Point3f(hw, hh, 0),   // TR
            cv::Point3f(hw, -hh, 0),  // BR
            cv::Point3f(-hw, -hh, 0)  // BL
        };

        kernel_open_ = cv::Mat::ones(3, 3, CV_8U);
        kernel_close_ = cv::Mat::ones(7, 7, CV_8U);

        timer_ = create_wall_timer(std::chrono::milliseconds(33), std::bind(&PnPNode::process, this));
        RCLCPP_INFO(this->get_logger(), "PnP C++ Node started.");
    }

private:
    std::vector<cv::Point2f> order_points(const cv::Point2f pts[4]) {
        std::vector<cv::Point2f> rect(4);
        std::vector<float> sums(4), diffs(4);
        for(int i = 0; i < 4; ++i) {
            sums[i] = pts[i].x + pts[i].y;
            diffs[i] = pts[i].y - pts[i].x;
        }
        rect[0] = pts[std::distance(sums.begin(), std::min_element(sums.begin(), sums.end()))];   // TL
        rect[2] = pts[std::distance(sums.begin(), std::max_element(sums.begin(), sums.end()))];   // BR
        rect[1] = pts[std::distance(diffs.begin(), std::min_element(diffs.begin(), diffs.end()))]; // TR
        rect[3] = pts[std::distance(diffs.begin(), std::max_element(diffs.begin(), diffs.end()))]; // BL
        return rect;
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
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(10, 50, 30), cv::Scalar(40, 255, 200), mask);

        // Morphology để khử nhiễu (Giống Python)
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_open_, cv::Point(-1,-1), 1);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_close_, cv::Point(-1,-1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool detected = false;

        if (!contours.empty()) {
            auto biggest = *std::max_element(contours.begin(), contours.end(), 
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            if (cv::contourArea(biggest) > 500) {
                cv::RotatedRect rect = cv::minAreaRect(biggest);
                cv::Point2f pts[4]; rect.points(pts);
                std::vector<cv::Point2f> img_pts = order_points(pts);

                // Vẽ viền chữ nhật màu vàng
                std::vector<cv::Point> int_img_pts(4);
                for(int i=0; i<4; i++) int_img_pts[i] = img_pts[i];
                cv::polylines(frame, int_img_pts, true, cv::Scalar(0, 255, 255), 2);

                cv::Mat rvec, tvec;
                bool success = false;
                
                if (has_prev_extrinsic_) {
                    rvec = prev_rvec_.clone();
                    tvec = prev_tvec_.clone();
                    success = cv::solvePnP(obj_pts_, img_pts, K_, dist_, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
                } else {
                    success = cv::solvePnP(obj_pts_, img_pts, K_, dist_, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
                }

                if (success) {
                    // Check Reprojection Error
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

                        // Chuyển rvec sang Quaternion để SLERP
                        cv::Mat R;
                        cv::Rodrigues(rvec, R);
                        tf2::Matrix3x3 tf2_R(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                             R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                             R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
                        tf2::Quaternion rot_cur;
                        tf2_R.getRotation(rot_cur);

                        // Reset smooth nếu nhảy xa
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
                            // SLERP cho Rotation và LERP cho Translation
                            rot_smooth_ = rot_smooth_.slerp(rot_cur, 1.0 - alpha_);
                            tvec_smooth_ = alpha_ * tvec_smooth_ + (1.0 - alpha_) * tvec;
                        }

                        publish_data(frame, rot_smooth_, tvec_smooth_);
                        detected = true;
                    }
                }
            }
        }

        if (!detected) {
            has_prev_extrinsic_ = false;
            cv::putText(frame, "NO DETECTION", cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        image_pub_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg());
        mask_pub_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg());
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
        cv::line(frame, imgpts[0], imgpts[1], cv::Scalar(0,0,255), 3); // X đỏ
        cv::line(frame, imgpts[0], imgpts[2], cv::Scalar(0,255,0), 3); // Y xanh lá
        cv::line(frame, imgpts[0], imgpts[3], cv::Scalar(255,0,0), 3); // Z xanh dương

        // Ghi Text
        char text[100];
        sprintf(text, "x=%.3f y=%.3f z=%.3f", t_msg.x, t_msg.y, t_msg.z);
        cv::putText(frame, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_, mask_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr tvec_pub_, rvec_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    cv::Mat latest_frame_, K_, dist_, kernel_open_, kernel_close_;
    std::vector<cv::Point3f> obj_pts_;

    bool has_prev_extrinsic_ = false;
    cv::Mat prev_rvec_, prev_tvec_;
    
    bool has_smooth_ = false;
    tf2::Quaternion rot_smooth_;
    cv::Mat tvec_smooth_;
    double alpha_ = 0.3;
    double max_jump_ = 0.15;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PnPNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}