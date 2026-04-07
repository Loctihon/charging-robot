#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>  // Thư viện bắt buộc để đọc Encoder từ Gazebo
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm> // Bắt buộc cho hàm std::clamp

class SwerveController : public rclcpp::Node
{
public:
    SwerveController() : Node("swerve_controller")
    {
        L_ = 1.476; // Chiều dài cơ sở (Khoảng cách giữa 2 trục bánh xe)
        W_ = 0.510; // Chiều rộng cơ sở
        wheel_radius_ = 0.125; // Bán kính bánh xe (m)
        wheel_positions_ = {
            { L_/2,  W_/2},   // Bánh Trước - Trái (FL)
            { L_/2, -W_/2},   // Bánh Trước - Phải (FR)
            {-L_/2,  W_/2},   // Bánh Sau - Trái (RL)
            {-L_/2, -W_/2}    // Bánh Sau - Phải (RR)
        };
        last_angles_ = std::vector<double>(4, 0.0);
        last_time_ = now();

        // Nhận lệnh vận tốc (từ MPPI Planner hoặc Teleop Keyboard)
        sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10,
            std::bind(&SwerveController::cmdCallback, this, std::placeholders::_1));
            
        // Nhận dữ liệu thực tế (Real Odom) từ Encoder của 4 bánh xe
        sub_joint_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&SwerveController::jointCallback, this, std::placeholders::_1));

        // Xuất lệnh điều khiển xuống các motor lái và motor lăn
        pub_steering_ = create_publisher<std_msgs::msg::Float64MultiArray>("/steering_controller/commands", 10);
        pub_drive_ = create_publisher<std_msgs::msg::Float64MultiArray>("/drive_controller/commands", 10);
            
        // Xuất dữ liệu Odometry (Vị trí) lên cho Nav2/AMCL
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Chạy ở tần số 50Hz (20ms) để cung cấp dữ liệu siêu mượt cho bộ dự báo MPPI
        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SwerveController::updateOdom, this));

        RCLCPP_INFO(get_logger(), "Swerve Controller PRO (REAL ODOM + MPPI OPTIMIZED) READY");
    }

private:
    double L_, W_, wheel_radius_;
    std::vector<std::pair<double,double>> wheel_positions_;

    double x_=0, y_=0, theta_=0;
    
    // Vận tốc ĐẦU VÀO (Mục tiêu từ Planner truyền xuống)
    double cmd_vx_=0, cmd_vy_=0, cmd_w_=0; 
    
    // Vận tốc THỰC TẾ (Tính toán ngược từ Encoder bánh xe bằng Động học thuận)
    double real_vx_=0, real_vy_=0, real_w_=0; 

    std::vector<double> last_angles_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_steering_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_drive_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Hàm chuẩn hóa góc xoay luôn nằm trong khoảng [-PI, PI]
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;
        return angle;
    }

    // CALLBACK 1: NHẬN LỆNH ĐIỀU KHIỂN TỪ NAV2 VÀ TÍNH TOÁN ĐỘNG HỌC NGƯỢC (INVERSE KINEMATICS)
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double target_vx = msg->linear.x;
        double target_vy = msg->linear.y;
        double target_w  = msg->angular.z;

        if (target_vx == 0.0 && target_vy == 0.0 && target_w == 0.0) {
            cmd_vx_ = 0.0;
            cmd_vy_ = 0.0;
            cmd_w_  = 0.0;
        } else {
            double max_acc_lin = 1.0 * 0.1; 
            double max_acc_ang = 2.0 * 0.1; 

            cmd_vx_ = std::clamp(target_vx, real_vx_ - max_acc_lin, real_vx_ + max_acc_lin);
            cmd_vy_ = std::clamp(target_vy, real_vy_ - max_acc_lin, real_vy_ + max_acc_lin);
            cmd_w_  = std::clamp(target_w,  real_w_  - max_acc_ang, real_w_  + max_acc_ang);
        }

        std_msgs::msg::Float64MultiArray steer_msg;
        std_msgs::msg::Float64MultiArray drive_msg;
        std::vector<double> speeds(4);
        std::vector<double> angles(4);
        double max_speed = 0.0;

        for (int i = 0; i < 4; i++)
        {
            double x = wheel_positions_[i].first;
            double y = wheel_positions_[i].second;

            // Tính vận tốc riêng cho từng bánh (Inverse Kinematics)
            double vx_w = cmd_vx_ - cmd_w_ * y;
            double vy_w = cmd_vy_ + cmd_w_ * x;

            double speed = std::hypot(vx_w, vy_w);
            double angle = std::atan2(vy_w, vx_w);


            // ---------------------------------------------------------
            if (std::fabs(speed) < 0.02) {
                angle = last_angles_[i]; // Giữ nguyên góc lái cũ
                speed = 0.0;             // Khóa chết động cơ lăn
            } else {
                double delta = normalizeAngle(angle - last_angles_[i]);
                if (std::fabs(delta) > M_PI/2)
                {
                    angle = normalizeAngle(angle + M_PI);
                    speed = -speed;
                }
            }

            last_angles_[i] = angle;
            double wheel_omega = speed / wheel_radius_;
            speeds[i] = wheel_omega;
            angles[i] = angle;

            if (std::fabs(wheel_omega) > max_speed)
                max_speed = std::fabs(wheel_omega);
        }

        // Bình thường hóa vận tốc (Normalize): Nếu bánh chạy vượt giới hạn, giảm tốc đều 4 bánh
        double max_allowed = 12.0;
        if (max_speed > max_allowed)
        {
            double scale = max_allowed / max_speed;
            for (auto &s : speeds) s *= scale;
        }

        for (int i = 0; i < 4; i++)
        {
            steer_msg.data.push_back(angles[i]);
            drive_msg.data.push_back(speeds[i]);
        }
        pub_steering_->publish(steer_msg);
        pub_drive_->publish(drive_msg);
    }

    // CALLBACK 2: ĐỌC ODOM THẬT (REAL ODOM) TỪ GAZEBO VÀ TÍNH TOÁN ĐỘNG HỌC THUẬN (FORWARD KINEMATICS)
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->velocity.empty()) return; 

        int id_steer[4] = {-1, -1, -1, -1};
        int id_drive[4] = {-1, -1, -1, -1};
        std::string steer_names[4] = {"Revolute1_to_base_link", "Revolute2_to_base_link", "Revolute3_to_base_link", "Revolute4_to_base_link"};
        std::string drive_names[4] = {"Wheel1_to_Revolute1", "Wheel2_to_Revolute2", "Wheel3_to_Revolute3", "Wheel4_to_Revolute4"};

        // 1. AUTO-MAPPING: Tự động dò tìm đúng khớp của bánh xe, bỏ qua các khớp của cánh tay UR10
        for (size_t i = 0; i < msg->name.size(); i++) {
            for (int j = 0; j < 4; j++) {
                if (msg->name[i] == steer_names[j]) id_steer[j] = i;
                if (msg->name[i] == drive_names[j]) id_drive[j] = i;
            }
        }

        for (int j = 0; j < 4; j++) {
            if (id_steer[j] == -1 || id_drive[j] == -1) return; 
        }

        double sum_vx = 0.0, sum_vy = 0.0, sum_omega_num = 0.0, sum_omega_den = 0.0;

        // 2. FORWARD KINEMATICS: Tổng hợp vector vận tốc của xe từ 4 bánh xe
        for (int i = 0; i < 4; i++) {
            double delta = msg->position[id_steer[i]]; // Góc lái thực (Radian)
            double v_wheel = msg->velocity[id_drive[i]] * wheel_radius_; // Vận tốc lăn thực (m/s)

            double vx_i = v_wheel * cos(delta);
            double vy_i = v_wheel * sin(delta);

            sum_vx += vx_i;
            sum_vy += vy_i;

            double x_i = wheel_positions_[i].first;
            double y_i = wheel_positions_[i].second;

            // Tính Omega_Z dựa trên phương pháp Bình Phương Tối Thiểu (Least-Squares) để khử sai số ma sát
            sum_omega_num += (vy_i * x_i - vx_i * y_i);
            sum_omega_den += (x_i * x_i + y_i * y_i);
        }

        // Trích xuất góc quay thô
        double raw_w = sum_omega_num / sum_omega_den;


        // Lọc bỏ nhiễu từ Encoder trước khi đẩy lên cho MPPI
        double alpha_lin = 0.3; // Tịnh tiến ít nhiễu hơn nên hệ số lớn
        double alpha_ang = 0.15; // Xoay nhiều nhiễu hơn do ma sát nên hệ số nhỏ hơn
        
        real_vx_ = alpha_lin * (sum_vx / 4.0) + (1 - alpha_lin) * real_vx_;
        real_vy_ = alpha_lin * (sum_vy / 4.0) + (1 - alpha_lin) * real_vy_;
        real_w_  = alpha_ang * raw_w + (1 - alpha_ang) * real_w_;


        // Lọc sai số khiến xe bị trôi nhẹ khi đứng yên (Deadzone cho Odom thực)
        if (std::fabs(real_vx_) < 0.01) real_vx_ = 0.0;
        if (std::fabs(real_vy_) < 0.01) real_vy_ = 0.0;
        if (std::fabs(real_w_)  < 0.01) real_w_  = 0.0;

        real_w_ = std::clamp(real_w_, -3.0, 3.0); // Khóa trần vận tốc xoay
    }

    // ODOMETRY PUBLISHER: TÍCH PHÂN TỌA ĐỘ VÀ ĐẨY LÊN TF TREE
    void updateOdom()
    {
        auto now_time = now();
        double dt = (now_time - last_time_).seconds();
        if (dt < 1e-4) return;
        last_time_ = now_time;

        double dx = (real_vx_*cos(theta_) - real_vy_*sin(theta_)) * dt;
        double dy = (real_vx_*sin(theta_) + real_vy_*cos(theta_)) * dt;
        double dth = real_w_ * dt;

        x_ += dx;
        y_ += dy;
        theta_ += dth;

        tf2::Quaternion q;
        q.setRPY(0,0,theta_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // Ép Real Velocity (Odom thật) vào bản tin để MPPI tính toán dự báo
        odom.twist.twist.linear.x = real_vx_;
        odom.twist.twist.linear.y = real_vy_;
        odom.twist.twist.angular.z = real_w_;
        
        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwerveController>());
    rclcpp::shutdown();
    return 0;
}