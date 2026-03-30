#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>

class SwerveController : public rclcpp::Node
{
public:
    SwerveController() : Node("swerve_controller"), x_(0.0), y_(0.0), theta_(0.0), current_vx_(0.0), current_vy_(0.0), current_omega_(0.0)
    {
        // --- THÔNG SỐ ROBOT ---
        L_ = 1.476;   
        W_ = 0.510;
        wheel_radius_ = 0.125;

        // [FL, FR, RL, RR]
        wheel_positions_ = {
            { L_ / 2.0,  W_ / 2.0}, 
            { L_ / 2.0, -W_ / 2.0}, 
            {-L_ / 2.0,  W_ / 2.0}, 
            {-L_ / 2.0, -W_ / 2.0}  
        };

        last_time_ = this->now();

        // Publishers & Subscribers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, std::bind(&SwerveController::listener_callback, this, std::placeholders::_1));
            
        pub_steering_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/steering_controller/commands", 10);
        pub_drive_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drive_controller/commands", 10);

        // Timer cập nhật Odom (20Hz -> 50ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&SwerveController::update_odometry, this));

        RCLCPP_INFO(this->get_logger(), "Swerve Open-Loop Controller (Bản Gốc Ổn Định) đã khởi động!");
    }

private:
    double L_, W_, wheel_radius_;
    std::vector<std::pair<double, double>> wheel_positions_;
    double x_, y_, theta_;
    rclcpp::Time last_time_;
    double current_vx_, current_vy_, current_omega_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_steering_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_drive_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_vx_ = msg->linear.x;
        current_vy_ = msg->linear.y;
        current_omega_ = msg->angular.z;

        std_msgs::msg::Float64MultiArray msg_steer;
        std_msgs::msg::Float64MultiArray msg_drive;

        std::vector<double> speeds(4);
        std::vector<double> angles(4);
        double max_speed = 0.0;

        for (int i = 0; i < 4; i++) {
            double x_pos = wheel_positions_[i].first;
            double y_pos = wheel_positions_[i].second;

            // Động học ngược chuẩn ROS 2
            double wheel_vx = current_vx_ - current_omega_ * y_pos;
            double wheel_vy = current_vy_ + current_omega_ * x_pos;

            double target_linear_speed = std::hypot(wheel_vx, wheel_vy);
            double target_angle = std::atan2(wheel_vy, wheel_vx);

            double target_angular_speed = target_linear_speed / wheel_radius_;

            speeds[i] = target_angular_speed;
            angles[i] = target_angle;

            if (std::abs(target_angular_speed) > max_speed) {
                max_speed = std::abs(target_angular_speed);
            }
        }

        // Tối ưu NORMALIZE (Giúp xe không lạng đuôi khi chạy nhanh)
        double max_allowed = 10.0;
        if (max_speed > max_allowed) {
            double scale = max_allowed / max_speed;
            for (auto &s : speeds) {
                s *= scale;
            }
        }

        for (int i = 0; i < 4; i++) {
            msg_steer.data.push_back(angles[i]);
            msg_drive.data.push_back(speeds[i]);
        }

        pub_steering_->publish(msg_steer);
        pub_drive_->publish(msg_drive);
    }

    void update_odometry()
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Dead Reckoning (Odom ảo nhưng siêu mượt cho giả lập)
        double delta_x = (current_vx_ * std::cos(theta_) - current_vy_ * std::sin(theta_)) * dt;
        double delta_y = (current_vx_ * std::sin(theta_) + current_vy_ * std::cos(theta_)) * dt;
        double delta_th = current_omega_ * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_th;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        // Publish Odometry
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

        odom.twist.twist.linear.x = current_vx_;
        odom.twist.twist.linear.y = current_vy_;
        odom.twist.twist.angular.z = current_omega_;

        odom_pub_->publish(odom);

        // Publish TF
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