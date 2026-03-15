import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import sys

class GoToXYNode(Node):
    def __init__(self):
        super().__init__('goto_xy_node')
        
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Vị trí hiện tại
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Đích đến
        self.target_x = None
        self.target_y = None
        
        # Cờ trạng thái
        self.has_odom = False
        self.goal_reached = True

        # Timer điều khiển (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.has_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Đổi Quaternion sang Euler (Yaw) để biết góc quay xe
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def set_goal(self, x, y):
        self.target_x = float(x)
        self.target_y = float(y)
        self.goal_reached = False
        self.get_logger().info(f"Nhận lệnh: Đi tới x={x}, y={y}")

    def control_loop(self):
        # Nếu chưa có Odom hoặc chưa có mục tiêu -> nghỉ
        if not self.has_odom or self.goal_reached:
            return

        # 1. Tính khoảng cách tới đích
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx**2 + dy**2)

        # 2. Tính góc cần quay tới đích
        angle_to_goal = math.atan2(dy, dx)
        
        # 3. Tính độ lệch góc (Heading Error)
        angle_error = angle_to_goal - self.yaw
        
        # Chuẩn hóa góc về khoảng -PI đến PI (để nó quay đường ngắn nhất)
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        msg = Twist()

        # --- LOGIC ĐIỀU KHIỂN (Proportional Controller) ---
        
        # Nếu đã đến rất gần đích (sai số < 5cm) -> Dừng
        if distance < 0.05:
            self.goal_reached = True
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub_vel.publish(msg)
            print(f"\n✅ Đã đến nơi! (Vị trí: {self.x:.2f}, {self.y:.2f})")
            print("Nhập toạ độ tiếp theo đi bro: ")
            return

        # Nếu góc lệch quá lớn (> 20 độ) -> Quay tại chỗ cho chuẩn trước
        if abs(angle_error) > 0.35:
            msg.linear.x = 0.0
            msg.angular.z = 0.8 * angle_error # Quay nhanh dần nếu lệch nhiều
        else:
            # Góc đã tạm ổn -> Vừa đi vừa chỉnh (Lái lụa)
            msg.linear.x = 0.3  # Tốc độ đi thẳng
            msg.angular.z = 1.5 * angle_error # Vừa đi vừa bẻ lái nhẹ

        self.pub_vel.publish(msg)

def main():
    rclpy.init()
    node = GoToXYNode()

    # Chạy một luồng riêng để lắng nghe ROS (Spinning)
    # Vì input() sẽ chặn dòng lệnh, nên ta dùng cách thủ công này
    
    print("ROBOT SẴN SÀNG! Đợi Odom khởi tạo...")
    
    # Đợi Odom nhận tín hiệu lần đầu
    while rclpy.ok() and not node.has_odom:
        rclpy.spin_once(node)

    print("Đã kết nối Odom.")
    print("Nhập tọa độ theo cú pháp: x y (Ví dụ: 2.0 1.5)")
    print("Nhập 'q' để thoát.")

    try:
        while rclpy.ok():
            # Xử lý các callback của ROS (cập nhật vị trí, gửi lệnh twist)
            rclpy.spin_once(node, timeout_sec=0.1)

            # Nếu robot đang rảnh (đã đến đích), cho nhập lệnh mới
            if node.goal_reached:
                try:
                    user_input = input("Nhập toạ độ (x y) > ")
                    if user_input.lower() == 'q':
                        break
                    
                    parts = user_input.split()
                    if len(parts) == 2:
                        node.set_goal(parts[0], parts[1])
                    else:
                        print("Nhập sai! Phải nhập 2 số cách nhau bởi dấu cách.")
                except ValueError:
                    print("Lỗi: Toạ độ phải là số.")
                    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()