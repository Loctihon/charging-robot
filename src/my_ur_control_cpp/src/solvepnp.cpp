#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>

class PnPNode : public rclcpp::Node {
public:
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
                "LỖI: Không tìm thấy ảnh congsac2.png! Copy ra thư mục chạy lệnh ros2 run nha!");
            return;
        }

        // 3. MULTI-SCALE MATCHING
        double best_max_val = -1.0;
        cv::Point best_max_loc;
        double best_scale = 1.0;

        for (double scale = 0.2; scale <= 2.0; scale += 0.1) {
            int new_w = (int)(template_img.cols * scale);
            int new_h = (int)(template_img.rows * scale);

            if (new_w >= edges_frame.cols || new_h >= edges_frame.rows) continue;

            cv::Mat resized_template, edges_template;
            cv::resize(template_img, resized_template, cv::Size(new_w, new_h));
            cv::Canny(resized_template, edges_template, 20, 80);

            cv::Mat result_matrix;
            cv::matchTemplate(edges_frame, edges_template, result_matrix, cv::TM_CCOEFF_NORMED);

            double min_val, max_val;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(result_matrix, &min_val, &max_val, &min_loc, &max_loc);

            if (max_val > best_max_val) {
                best_max_val = max_val;
                best_max_loc = max_loc;
                best_scale = scale;
            }
        }

        bool detected = false;

        // 4. NGƯỠNG TIN CẬY
        if (best_max_val > 0.25) {
            int final_w = (int)(template_img.cols * best_scale);
            int final_h = (int)(template_img.rows * best_scale);

            cv::Point2f pt_TL(best_max_loc.x, best_max_loc.y);               
            cv::Point2f pt_TR(best_max_loc.x + final_w, best_max_loc.y);           
            cv::Point2f pt_BR(best_max_loc.x + final_w, best_max_loc.y + final_h);       
            cv::Point2f pt_BL(best_max_loc.x, best_max_loc.y + final_h);           

            std::vector<cv::Point2f> img_pts = {pt_TL, pt_TR, pt_BR, pt_BL};

            std::vector<cv::Point> int_img_pts = {pt_TL, pt_TR, pt_BR, pt_BL};
            cv::polylines(frame, int_img_pts, true, cv::Scalar(0, 255, 255), 2);
            
            char text_info[50];
            sprintf(text_info, "Shape: %.2f | Scale: %.1f", best_max_val, best_scale);
            cv::putText(frame, text_info, pt_TL, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            // 5. SOLVEPNP
            cv::Mat rvec, tvec;
            bool success = false;
            
            if (has_prev_extrinsic_) {
                rvec = prev_rvec_.clone();
                tvec = prev_tvec_.clone();
                success = cv::solvePnP(obj_pts_, img_pts, K_, dist_, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
            } else {
                success = cv::solvePnP(obj_pts_, img_pts, K_, dist_, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
            }

            // 6. SLERP SMOOTHING
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

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PnPNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}