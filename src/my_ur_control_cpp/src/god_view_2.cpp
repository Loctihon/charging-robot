#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>
#include <thread>
#include <iostream>
#include <algorithm> 


struct Pose2D { double x, y, yaw; };
struct Station { Pose2D approach, dock; };

class AutoDockingNode : public rclcpp::Node {
public:
    using Nav2Pose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<Nav2Pose>;

    AutoDockingNode() : Node("auto_docking_node") {
        // Khai báo parameter PID (Giống y hệt Python)
        this->declare_parameter("Kp", 1.0);
        this->declare_parameter("Ki", 0.05);
        this->declare_parameter("Kd", 0.1);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        nav_client_ = rclcpp_action::create_client<Nav2Pose>(this, "navigate_to_pose");
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        stations_["1"] = {{7.5, 0.0, 0.0}, {8.5, -0.3, 1.57}};  
        stations_["2"] = {{7.5, 4.0, 0.0}, {8.5, 3.7, 1.57}};  
        stations_["3"] = {{0.0, 7.5, 1.57}, {0.3, 8.5, 3.14}}; 

        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&AutoDockingNode::control_loop, this));
        
        std::thread([this]() {
            while (rclcpp::ok()) {
                std::string input;
                std::cout << "\n[NHẬP LỆNH] Nhập số trạm sạc (1, 2, 3): "; 
                std::cin >> input;
                
                if (stations_.find(input) != stations_.end() && state_ == "IDLE") {
                    start_mission(stations_[input], input);
                } else if (state_ != "IDLE") {
                    std::cout << "[CẢNH BÁO] Xe đang bận chạy nhiệm vụ!\n";
                } else {
                    std::cout << "[LỖI] Trạm không tồn tại!\n";
                }
            }
        }).detach();
    }

private:
    void start_mission(const Station& station, const std::string& station_id) {
        state_ = "NAV2";
        goal_x_ = station.dock.x;
        goal_y_ = station.dock.y;
        goal_yaw_ = station.dock.yaw;

        RCLCPP_INFO(this->get_logger(), "Đang gửi lệnh tới Nav2: Đi đến điểm chờ Trạm %s...", station_id.c_str());

        auto goal_msg = Nav2Pose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = station.approach.x;
        goal_msg.pose.pose.position.y = station.approach.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, station.approach.yaw); 
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

        auto send_goal_options = rclcpp_action::Client<Nav2Pose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&AutoDockingNode::nav2_goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&AutoDockingNode::nav2_result_callback, this, std::placeholders::_1);
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void nav2_goal_response_callback(const GoalHandleNav2::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 TỪ CHỐI lệnh!");
            state_ = "IDLE";
        } else {
            RCLCPP_INFO(this->get_logger(), "Nav2 CHẤP NHẬN lệnh. Bắt đầu di chuyển...");
        }
    }

    void nav2_result_callback(const GoalHandleNav2::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Tới điểm chờ thành công! Bắt đầu trượt lùi chuồng...");
            state_ = "PID";
            pid_step_ = 0;
            sum_err_x_ = 0.0; sum_err_y_ = 0.0;
            current_vx_ = 0.0; current_vy_ = 0.0; current_vz_ = 0.0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Nav2 BỊ HỦY HOẶC LỖI. Hủy lệnh sạc!");
            state_ = "IDLE";
        }
    }


    void control_loop() {
        if (state_ != "PID") return;

        geometry_msgs::msg::TransformStamped trans;
        try {
            trans = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return; 
        }

        double curr_x = trans.transform.translation.x;
        double curr_y = trans.transform.translation.y;
        double curr_yaw = tf2::getYaw(trans.transform.rotation); 

        double Kp = this->get_parameter("Kp").as_double();
        double Ki = this->get_parameter("Ki").as_double();
        double Kd = this->get_parameter("Kd").as_double();

        double dx_global = goal_x_ - curr_x;
        double dy_global = goal_y_ - curr_y;
        
        double dyaw = goal_yaw_ - curr_yaw;
        dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));

        double local_dx = std::cos(curr_yaw) * dx_global + std::sin(curr_yaw) * dy_global;
        double local_dy = -std::sin(curr_yaw) * dx_global + std::cos(curr_yaw) * dy_global;

        double target_vx = 0.0, target_vy = 0.0, target_vz = 0.0;

        if (pid_step_ == 0) {
            if (std::abs(dyaw) > 0.01) {
                target_vz = std::clamp(2.0 * dyaw, -0.2, 0.2);
            } else {
                pid_step_ = 1; 
                sum_err_x_ = 0.0;
            }
        } 
        else if (pid_step_ == 1) {
            if (std::abs(local_dx) > 0.005) {
                double p_x = Kp * local_dx;
                double d_x = Kd * ((local_dx - prev_err_x_) / dt_);
                double i_x = 0.0;
                
                // Anti-windup
                if (std::abs(local_dx) < 0.2) {
                    sum_err_x_ = std::clamp(sum_err_x_ + local_dx * dt_, -0.5, 0.5);
                    i_x = Ki * sum_err_x_;
                } else {
                    sum_err_x_ = 0.0;
                }
                target_vx = std::clamp(p_x + i_x + d_x, -0.15, 0.15);
            } else {
                pid_step_ = 2;
                sum_err_y_ = 0.0;
            }
        } 
        else if (pid_step_ == 2) {
            if (std::abs(local_dy) > 0.005) {
                double p_y = (Kp * 2.5) * local_dy;   
                double d_y = Kd * ((local_dy - prev_err_y_) / dt_);
                double i_y = 0.0;

                // Anti-windup
                if (std::abs(local_dy) < 0.2) {
                    sum_err_y_ = std::clamp(sum_err_y_ + local_dy * dt_, -0.5, 0.5);
                    i_y = Ki * sum_err_y_;
                } else {
                    sum_err_y_ = 0.0;
                }
                target_vy = std::clamp(p_y + i_y + d_y, -0.1, 0.1);
            } else {
                // ĐẾN ĐÍCH: Phanh gấp và tắt máy
                cmd_pub_->publish(geometry_msgs::msg::Twist()); 
                RCLCPP_INFO(this->get_logger(), "HẾT HỒN CHƯA:)) Đỗ xe thành công!");
                state_ = "IDLE";
                return;
            }
        }

        prev_err_x_ = local_dx;
        prev_err_y_ = local_dy;

        // Giới hạn gia tốc (Max Accel)
        double max_accel = 0.015; 
        current_vx_ += std::clamp(target_vx - current_vx_, -max_accel, max_accel);
        current_vy_ += std::clamp(target_vy - current_vy_, -max_accel, max_accel);
        current_vz_ += std::clamp(target_vz - current_vz_, -max_accel, max_accel);

        // Gửi lệnh xuống bánh xe
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = current_vx_;
        cmd.linear.y = current_vy_;
        cmd.angular.z = current_vz_;
        cmd_pub_->publish(cmd);
    }

    std::string state_ = "IDLE";
    std::map<std::string, Station> stations_; 
    double goal_x_ = 0.0, goal_y_ = 0.0, goal_yaw_ = 0.0; 
    
    int pid_step_ = 0;
    double sum_err_x_ = 0.0, sum_err_y_ = 0.0;
    double prev_err_x_ = 0.0, prev_err_y_ = 0.0;
    double current_vx_ = 0.0, current_vy_ = 0.0, current_vz_ = 0.0;
    double dt_ = 0.1;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Client<Nav2Pose>::SharedPtr nav_client_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoDockingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}