#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <map>
#include <thread>
#include <iostream>

struct Pose2D { double x, y, yaw; };

class AutoDockingNode : public rclcpp::Node {
public:
    using Nav2Pose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<Nav2Pose>;

    AutoDockingNode() : Node("auto_docking_node", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})) {
        
        nav_client_ = rclcpp_action::create_client<Nav2Pose>(this, "navigate_to_pose");
        
        // ĐÃ DỌN DẸP SẠCH SẼ: Chỉ còn đúng 1 tọa độ Đích cho mỗi trạm (x, y, yaw)
        stations_["1"] = {-8.0, 0.0, 1.57};  
        stations_["2"] = {-8.0, -4.0, 1.57};  
        stations_["3"] = {0.0, -8.0, 3.14}; 

        
        RCLCPP_INFO(this->get_logger(), "Đang kết nối với Nav2 Action Server...");
        while (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Vẫn đang đợi Nav2 thức dậy...");
        }
        RCLCPP_INFO(this->get_logger(), ">>> NAV2 ĐÃ SẴN SÀNG! <<<");

        std::thread([this]() {
            while (rclcpp::ok()) {
                std::string input;
                std::cout << "\n[BẢNG ĐIỀU KHIỂN] Nhập số trạm sạc (1, 2, 3): "; 
                std::cin >> input;
                
                if (stations_.find(input) != stations_.end() && state_ == "IDLE") {
                    start_mission(stations_[input], input);
                } else if (state_ != "IDLE") {
                    std::cout << "[CẢNH BÁO] Xe đang bận! Chờ xíu đại ca!\n";
                } else {
                    std::cout << "[LỖI] Trạm này chưa xây!\n";
                }
            }
        }).detach();
    }

private:
    void start_mission(const Pose2D& target_pose, const std::string& station_id) {
        state_ = "NAV2";
        mission_start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Đã nhận lệnh! Hướng thẳng tới trạm %s...", station_id.c_str());

        auto goal_msg = Nav2Pose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        
        // Nạp tọa độ vào Goal
        goal_msg.pose.pose.position.x = target_pose.x;
        goal_msg.pose.pose.position.y = target_pose.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, target_pose.yaw); 
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
            RCLCPP_ERROR(this->get_logger(), "Nav2 TỪ CHỐI lệnh! (Điểm Goal có thể kẹt vật cản)");
            state_ = "IDLE";
        } else {
            RCLCPP_INFO(this->get_logger(), "Nav2 CHẤP NHẬN lệnh! Đang vẽ đường...");
        }
    }

    void nav2_result_callback(const GoalHandleNav2::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            double total_time = (this->now() - mission_start_time_).seconds();
            RCLCPP_INFO(this->get_logger(), "✅ HẾT HỒN CHƯA! ĐỖ XE THÀNH CÔNG");
            RCLCPP_INFO(this->get_logger(), "⏱️ TỔNG THỜI GIAN HOÀN THÀNH: %.2f giây", total_time);     
            state_ = "IDLE";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Lỗi Nav2: Không thể tới đích. Hủy nhiệm vụ!");
            state_ = "IDLE";
        }
    }

    std::string state_ = "IDLE";
    std::map<std::string, Pose2D> stations_;  // <--- Đã sửa thành Pose2D thuần túy
    rclcpp::Time mission_start_time_; 
    rclcpp_action::Client<Nav2Pose>::SharedPtr nav_client_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoDockingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}