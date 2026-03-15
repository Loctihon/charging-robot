import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np

def euler_from_quaternion(x, y, z, w):
    """Chuyển đổi Quaternion sang Góc Euler (Yaw)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class SwerveGoToPosePID(Node):
    def __init__(self):
        super().__init__('swerve_go_to_pose_pid')

        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.05)
        self.declare_parameter('Kd', 0.1)

        #GIAO TIẾP ROS
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.dt = 0.1 # Chu kỳ 10Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0

        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None

        self.sum_err_x = 0.0   
        self.sum_err_y = 0.0   # dùng Khâu I 
        self.prev_err_x = 0.0  # Dùng khâu D 
        self.prev_err_y = 0.0  

        self.get_logger().info("Node Swerve Full PID đã sẵn sàng!")

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        q = msg.pose.orientation
        self.goal_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(f"Đã nhận mục tiêu mới!")

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.curr_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def control_loop(self):
        if self.goal_x is None:
            return

        # Đọc tham số PID liên tục (hỗ trợ đổi số lúc đang chạy bằng rqt)
        Kp = self.get_parameter('Kp').get_parameter_value().double_value
        Ki = self.get_parameter('Ki').get_parameter_value().double_value
        Kd = self.get_parameter('Kd').get_parameter_value().double_value

        # Tính toán sai số Global
        dx_global = self.goal_x - self.curr_x
        dy_global = self.goal_y - self.curr_y
        
        dyaw = self.goal_yaw - self.curr_yaw
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw)) # Chuẩn hóa về [-pi, pi]

        distance = math.hypot(dx_global, dy_global)
        cmd = Twist()

        if distance > 0.02 or abs(dyaw) > 0.03:
            local_dx = math.cos(self.curr_yaw) * dx_global + math.sin(self.curr_yaw) * dy_global
            local_dy = -math.sin(self.curr_yaw) * dx_global + math.cos(self.curr_yaw) * dy_global

            #KHÂU P
            p_x = Kp * local_dx
            p_y = Kp * local_dy

            # KHÂU I 
            self.sum_err_x += local_dx * self.dt
            self.sum_err_y += local_dy * self.dt
            
            self.sum_err_x = np.clip(self.sum_err_x, -2.0, 2.0) # Giới hạn tích lũy khâu tích phân I, tránh bị vọt lố
            self.sum_err_y = np.clip(self.sum_err_y, -2.0, 2.0)
            
            i_x = Ki * self.sum_err_x
            i_y = Ki * self.sum_err_y
            #KHÂU D 
            d_err_x = (local_dx - self.prev_err_x) / self.dt
            d_err_y = (local_dy - self.prev_err_y) / self.dt
            
            d_x = Kd * d_err_x
            d_y = Kd * d_err_y

            self.prev_err_x = local_dx
            self.prev_err_y = local_dy

            cmd.linear.x = np.clip(p_x + i_x + d_x, -0.4, 0.4)
            cmd.linear.y = np.clip(p_y + i_y + d_y, -0.4, 0.4) # tốc độ tối đa 0.4 m/s
            
            cmd.angular.z = np.clip(1.5 * dyaw, -0.5, 0.5) # Trục xoay

        else:
            self.get_logger().info("Đã đến đích")
            self.goal_x = None 
            # Reset biến tích phân khi đến đích
            self.sum_err_x = 0.0
            self.sum_err_y = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveGoToPosePID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()