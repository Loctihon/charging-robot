import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class TestDriveNode(Node):
    def __init__(self):
        super().__init__('test_drive_node')
        
        # Publisher gửi lệnh cmd_vel
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber nhận Odom để đếm mét
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.start_x = None
        self.start_y = None
        self.is_moving = False
        

        self.timer_period = 0.1  # 0.1 giây (10Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info("Test Drive Node: Sẵn sàng! Đang chờ Odom để bắt đầu...")

    def timer_callback(self):
 
        if self.is_moving:
            msg = Twist()
            msg.linear.x = 0.1 # Tốc độ
            self.pub_vel.publish(msg)


    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = current_x
            self.start_y = current_y
            self.is_moving = True 
            self.get_logger().info(f"Đã chốt vị trí gốc: x={current_x:.2f}, y={current_y:.2f}. BẮT ĐẦU CHẠY!")
            return

        if not self.is_moving:
            return

        distance = math.sqrt((current_x - self.start_x)**2 + (current_y - self.start_y)**2)
        self.get_logger().info(f"Đã đi: {distance:.3f} m", throttle_duration_sec=0.5)

        if distance >= 1.0:
            self.stop_robot()

    def stop_robot(self):
        self.get_logger().info("ĐỦ 1 MÉT ")
        self.is_moving = False 

        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        self.pub_vel.publish(stop_msg)
        self.pub_vel.publish(stop_msg)
        
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TestDriveNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass # Thoát êm đẹp

if __name__ == '__main__':
    main()