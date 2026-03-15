import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math
import numpy as np
import threading
from tf2_ros import Buffer, TransformListener

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def quaternion_from_yaw(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

class AutoDockingNode(Node):
    def __init__(self):
        super().__init__('auto_docking_node')

        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.05)
        self.declare_parameter('Kd', 0.1)

        self.state = "IDLE"
        self.pid_step = 0

        # --- LƯU Ý TỌA ĐỘ ---
        # 1. Approach: Lùi ra xa TRÊN 1.5 MÉT để Nav2 không đụng vùng xanh Inflation
        # 2. Dock: Lái xe bằng tay vô đậu thật đẹp -> gõ tf2_echo map base_footprint -> Điền số vô đây!
        self.stations = {
            # Đã lùi approach ra X=7.5 để cực kỳ an toàn cho Nav2
            '1': {'approach': (7.5, 0.0, 0.0), 'dock': (8.5, 0.0, 1.57)},
            '2': {'approach': (7.5, 4.0, 0.0), 'dock': (8.5, 4.0, 1.57)},
            '3': {'approach': (0.0, 7.5, 1.57), 'dock': (0.0, 8.5, 3.14)}
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.curr_x, self.curr_y, self.curr_yaw = 0.0, 0.0, 0.0
        self.goal_x, self.goal_y, self.goal_yaw = None, None, None
        
        self.sum_err_x, self.sum_err_y = 0.0, 0.0
        self.prev_err_x, self.prev_err_y = 0.0, 0.0
        self.current_vx, self.current_vy, self.current_vz = 0.0, 0.0, 0.0

        self.dt = 0.1 
        self.timer = self.create_timer(self.dt, self.pid_control_loop)
        
        self.get_logger().info("Ready")

        
        threading.Thread(target=self.terminal_input_loop, daemon=True).start()

    def terminal_input_loop(self):
        while rclpy.ok():
            station_id = input("\n[NHẬP LỆNH] Nhập số trạm sạc (1, 2, 3): ")
            if station_id in self.stations and self.state == "IDLE":
                self.start_docking_mission(station_id)

    def start_docking_mission(self, station_id):
        self.state = "NAV2_TRANSIT"
        appr_x, appr_y, appr_yaw = self.stations[station_id]['approach']
        self.goal_x, self.goal_y, self.goal_yaw = self.stations[station_id]['dock'] 

        self.get_logger().info(f"Đi tới Điểm chờ Trạm {station_id} (Đã lùi ra xa an toàn)...")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = appr_x
        goal_msg.pose.pose.position.y = appr_y
        
        q = quaternion_from_yaw(appr_yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = "IDLE"
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Tới điểm chờ thành công! Bắt đầu trượt lùi chuồng...')
            self.state = "PID_DOCKING"
            self.pid_step = 0 
            self.sum_err_x, self.sum_err_y = 0.0, 0.0 
            self.current_vx, self.current_vy, self.current_vz = 0.0, 0.0, 0.0
        else:
            self.get_logger().error(f'Nav2 TỪ CHỐI TIẾP TỤC. HỦY LỆNH SẠC!')
            self.state = "IDLE"

    def pid_control_loop(self):
        if self.state != "PID_DOCKING" or self.goal_x is None:
            return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.curr_x = trans.transform.translation.x
            self.curr_y = trans.transform.translation.y
            q = trans.transform.rotation
            self.curr_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        except Exception:
            return

        Kp = self.get_parameter('Kp').get_parameter_value().double_value
        Ki = self.get_parameter('Ki').get_parameter_value().double_value
        Kd = self.get_parameter('Kd').get_parameter_value().double_value

        dx_global = self.goal_x - self.curr_x
        dy_global = self.goal_y - self.curr_y
        dyaw = self.goal_yaw - self.curr_yaw
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))

        local_dx = math.cos(self.curr_yaw) * dx_global + math.sin(self.curr_yaw) * dy_global
        local_dy = -math.sin(self.curr_yaw) * dx_global + math.cos(self.curr_yaw) * dy_global

        target_vx, target_vy, target_vz = 0.0, 0.0, 0.0

        if self.pid_step == 0:
            if abs(dyaw) > 0.01: 
                target_vz = np.clip(2.0 * dyaw, -0.2, 0.2)
            else:
                self.pid_step = 1 
                self.sum_err_x = 0.0

        elif self.pid_step == 1:
            if abs(local_dx) > 0.005: 
                p_x = Kp * local_dx
                d_x = Kd * ((local_dx - self.prev_err_x) / self.dt)
                
                # --- ANTI-WINDUP: Chỉ bật Khâu I khi cách tâm dưới 20cm ---
                if abs(local_dx) < 0.2:
                    self.sum_err_x = np.clip(self.sum_err_x + local_dx * self.dt, -0.5, 0.5)
                    i_x = Ki * self.sum_err_x
                else:
                    self.sum_err_x = 0.0
                    i_x = 0.0

                target_vx = np.clip(p_x + i_x + d_x, -0.15, 0.15)
                target_vy, target_vz = 0.0, 0.0 
            else:
                self.pid_step = 2
                self.sum_err_y = 0.0

        elif self.pid_step == 2:
            if abs(local_dy) > 0.005:
                p_y = (Kp * 2.5) * local_dy   
                d_y = Kd * ((local_dy - self.prev_err_y) / self.dt)
                
                # --- ANTI-WINDUP: Bóp phanh khâu I, chống lật xe và tông tường ---
                if abs(local_dy) < 0.2:
                    self.sum_err_y = np.clip(self.sum_err_y + local_dy * self.dt, -0.5, 0.5)
                    i_y = Ki * self.sum_err_y
                else:
                    self.sum_err_y = 0.0
                    i_y = 0.0

                # Đã giới hạn tốc độ trượt ngang cực êm (0.1 m/s)
                target_vy = np.clip(p_y + i_y + d_y, -0.1, 0.1)
                target_vx, target_vz = 0.0, 0.0 
            else:
                self.cmd_pub.publish(Twist()) 
                self.get_logger().info("HẾT HỒN CHƯA:))")
                self.state = "IDLE"
                return

        self.prev_err_x = local_dx
        self.prev_err_y = local_dy

        max_accel = 0.015 
        self.current_vx += np.clip(target_vx - self.current_vx, -max_accel, max_accel)
        self.current_vy += np.clip(target_vy - self.current_vy, -max_accel, max_accel)
        self.current_vz += np.clip(target_vz - self.current_vz, -max_accel, max_accel)

        cmd = Twist()
        cmd.linear.x = self.current_vx
        cmd.linear.y = self.current_vy
        cmd.angular.z = self.current_vz
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()