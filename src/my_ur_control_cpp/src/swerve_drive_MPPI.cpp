#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

class SwerveController : public rclcpp::Node
{
public:
    SwerveController() : Node("swerve_controller"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // --- THÔNG SỐ ROBOT ---
        L_ = 1.476;   
        W_ = 0.510;
        wheel_radius_ = 0.125;
        
        // Vị trí 4 bánh [FL, FR, RL, RR] (X tới, Y trái)
        wheel_positions_ = {
            { L_ / 2.0,  W_ / 2.0}, 
            { L_ / 2.0, -W_ / 2.0}, 
            {-L_ / 2.0,  W_ / 2.0}, 
            {-L_ / 2.0, -W_ / 2.0}  
        };

        // Tên Joint phải khớp 100% với file combine_controller.yaml
        steering_joint_names_ = {"Revolute1_to_base_link", "Revolute2_to_base_link", "Revolute3_to_base_link", "Revolute4_to_base_link"};
        drive_joint_names_ = {"Wheel1_to_Revolute1", "Wheel2_to_Revolute2", "Wheel3_to_Revolute3", "Wheel4_to_Revolute4"};

        last_time_ = this->now();

        // Publishers & Subscribers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        pub_steering_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/steering_controller/commands", 10);
        pub_drive_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drive_controller/commands", 10);

        // NHẬN LỆNH TỪ NAV2 (MPPI)
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, std::bind(&SwerveController::cmd_vel_callback, this, std::placeholders::_1));

        // ĐỌC ENCODER TỪ GAZEBO (CHÌA KHÓA CHỐNG QUAY MÒNG MÒNG)
        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 50, std::bind(&SwerveController::joint_states_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Swerve Closed-Loop Controller đã khởi động! Sẵn sàng đọc Encoder.");
    }

private:
    double L_, W_, wheel_radius_;
    std::vector<std::pair<double, double>> wheel_positions_;
    std::vector<std::string> steering_joint_names_;
    std::vector<std::string> drive_joint_names_;
    
    double x_, y_, theta_;
    rclcpp::Time last_time_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_steering_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_drive_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // =================================================================================
    // 1. INVERSE KINEMATICS: Não bộ ra lệnh cho cơ bắp (Giữ nguyên thuật toán chuẩn)
    // =================================================================================
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double omega = msg->angular.z;

        std_msgs::msg::Float64MultiArray msg_steer;
        std_msgs::msg::Float64MultiArray msg_drive;

        for (const auto& pos : wheel_positions_) {
            double x_pos = pos.first;
            double y_pos = pos.second;

            double wheel_vx = vx - omega * y_pos;
            double wheel_vy = vy + omega * x_pos;

            double target_linear_speed = std::hypot(wheel_vx, wheel_vy);
            double target_angle = std::atan2(wheel_vy, wheel_vx);

            double target_angular_speed = target_linear_speed / wheel_radius_;
            
            // Giới hạn tốc độ
            if (target_angular_speed > 10.0) target_angular_speed = 10.0;
            if (target_angular_speed < -10.0) target_angular_speed = -10.0;

            msg_steer.data.push_back(target_angle);
            msg_drive.data.push_back(target_angular_speed);
        }

        pub_steering_->publish(msg_steer);
        pub_drive_->publish(msg_drive);
    }

    // =================================================================================
    // 2. FORWARD KINEMATICS (ĐỘNG HỌC THUẬN): Đọc cảm biến thực tế để tính tọa độ
    // =================================================================================
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::vector<double> steering_angles(4, 0.0);
        std::vector<double> drive_velocities(4, 0.0);
        bool all_found = true;

        // Bóc tách dữ liệu Encoder của 8 khớp
        for (size_t i = 0; i < 4; ++i) {
            auto it_steer = std::find(msg->name.begin(), msg->name.end(), steering_joint_names_[i]);
            auto it_drive = std::find(msg->name.begin(), msg->name.end(), drive_joint_names_[i]);

            if (it_steer != msg->name.end() && it_drive != msg->name.end()) {
                int idx_steer = std::distance(msg->name.begin(), it_steer);
                int idx_drive = std::distance(msg->name.begin(), it_drive);
                
                steering_angles[i] = msg->position[idx_steer];
                drive_velocities[i] = msg->velocity[idx_drive];
            } else {
                all_found = false;
            }
        }

        if (!all_found) return; // Nếu chưa đủ data thì bỏ qua

        // TÍNH TOÁN VẬN TỐC THỰC TẾ CỦA THÂN XE (V_x, V_y, Omega)
        double real_vx = 0.0, real_vy = 0.0, real_omega = 0.0;

        for (size_t i = 0; i < 4; ++i) {
            double v_wheel_linear = drive_velocities[i] * wheel_radius_; // v = omega * R
            double angle = steering_angles[i];

            double v_ix = v_wheel_linear * std::cos(angle);
            double v_iy = v_wheel_linear * std::sin(angle);

            real_vx += v_ix;
            real_vy += v_iy;

            // Tính omega_z bằng trung bình cộng công thức động học thuận
            double x_pos = wheel_positions_[i].first;
            double y_pos = wheel_positions_[i].second;
            double r_squared = x_pos*x_pos + y_pos*y_pos;
            
            real_omega += (-v_ix * y_pos + v_iy * x_pos) / r_squared;
        }

        // Lấy trung bình cộng của 4 bánh
        real_vx /= 4.0;
        real_vy /= 4.0;
        real_omega /= 4.0;

        // CẬP NHẬT TỌA ĐỘ VÀ PUBLISH (Chỉ chạy khi có Encoder mới)
        auto current_time = msg->header.stamp;
        double dt = (rclcpp::Time(current_time) - last_time_).seconds();
        last_time_ = current_time;

        if (dt > 0.0 && dt < 1.0) { // Lọc nhiễu thời gian
            double delta_x = (real_vx * std::cos(theta_) - real_vy * std::sin(theta_)) * dt;
            double delta_y = (real_vx * std::sin(theta_) + real_vy * std::cos(theta_)) * dt;
            double delta_th = real_omega * dt;

            x_ += delta_x;
            y_ += delta_y;
            theta_ += delta_th;
        }

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        // Phát ODOMETRY
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = real_vx;
        odom.twist.twist.linear.y = real_vy;
        odom.twist.twist.angular.z = real_omega;

        odom_pub_->publish(odom);

        // Phát TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwerveController>());
    rclcpp::shutdown();
    return 0;
}