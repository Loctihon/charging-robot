import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')
        self.L = 1.476 # Khoảng cách 2 bánh trước sau từ 0,738 đến -0,738
        self.W = 0.51  # Khoảng cách giữa 2 bánh ngang từ 0,255 đến -0.255
        
        self.wheel_positions = [
            ( self.L / 2,  self.W / 2), # trước trái
            ( self.L / 2, -self.W / 2), # trước phải
            (-self.L / 2,  self.W / 2), # sau trái
            (-self.L / 2, -self.W / 2)  # sau phải
        ]

        # Nghe lệnh từ teleop_keyboard
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10)

        # Gửi lệnh góc quay xuống Controller (YAML steering_controller)
        self.pub_steering = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', 10)

        # Gửi lệnh tốc độ lăn xuống Controller (YAML drive_controller)
        self.pub_drive = self.create_publisher(
            Float64MultiArray, '/drive_controller/commands', 10)

        self.get_logger().info("Swerve Controller đã khởi động!")

    def listener_callback(self, msg):
        #ĐỌC DỮ LIỆU ĐẦU VÀO
        vx = msg.linear.x      # Tốc độ theo x
        vy = msg.linear.y      # Tốc độ theo y
        omega = msg.angular.z  # Tốc độ quay theo z
        steering_commands = []
        drive_commands = []

        # Duyệt qua từng bánh xe để tính toán
        for i, (wx, wy) in enumerate(self.wheel_positions): 
            wheel_vx = vx - omega * wy
            wheel_vy = vy + omega * wx

            #Chuyển sang Tốc độ & Góc 
            target_speed = math.sqrt(wheel_vx**2 + wheel_vy**2)
            target_angle = math.atan2(wheel_vy, wheel_vx)

            if abs(target_angle) > math.pi / 2:
                target_angle = target_angle - math.pi if target_angle > 0 else target_angle + math.pi
                target_speed = -target_speed
            steering_commands.append(target_angle)
            drive_commands.append(target_speed)


        msg_steer = Float64MultiArray()
        msg_steer.data = steering_commands 
        self.pub_steering.publish(msg_steer)


        msg_drive = Float64MultiArray()
        msg_drive.data = drive_commands   
        self.pub_drive.publish(msg_drive)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()