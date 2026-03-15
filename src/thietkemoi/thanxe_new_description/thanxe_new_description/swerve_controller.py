import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import TransformStamped 
from tf2_ros import TransformBroadcaster
import math
import time

class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')
        
        # --- THÔNG SỐ ROBOT (Cần kiểm tra kỹ URDF) ---
        self.L = 1.8 # Chiều dài cơ sở
        self.W = 0.425  # Chiều rộng cơ sở
        
        self.wheel_radius = 0.125 
        
        self.wheel_positions = [
            ( self.L / 2,  self.W / 2), # trước trái
            ( self.L / 2, -self.W / 2), # trước phải
            (-self.L / 2,  self.W / 2), # sau trái
            (-self.L / 2, -self.W / 2)  # sau phải
        ]

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Lưu vận tốc hiện tại để tính toán Odom
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0

        # Publisher /odom và TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.pub_steering = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.pub_drive = self.create_publisher(Float64MultiArray, '/drive_controller/commands', 10)

        # Tạo Timer để cập nhật Odom liên tục (20Hz)
        self.create_timer(0.05, self.update_odometry)

        self.get_logger().info(f"Swerve Controller đã khởi động! (Bán kính bánh xe: {self.wheel_radius}m)")

    def listener_callback(self, msg):
        # 1. Lưu vận tốc vào biến toàn cục để hàm update_odometry dùng
        self.current_vx = msg.linear.x
        self.current_vy = msg.linear.y
        self.current_omega = msg.angular.z

        # 2. Tính toán động học bánh xe
        steering_commands = []
        drive_commands = []

        for i, (wx, wy) in enumerate(self.wheel_positions): 
            wheel_vx = self.current_vx - self.current_omega * wy
            wheel_vy = self.current_vy + self.current_omega * wx

            # Tính vận tốc dài (m/s) và góc lái
            target_linear_speed = math.sqrt(wheel_vx**2 + wheel_vy**2)
            target_angle = math.atan2(wheel_vy, wheel_vx)

            # Logic đảo chiều bánh xe tối ưu (để không phải quay quá 90 độ)
            if abs(target_angle) > math.pi / 2:
                target_angle = target_angle - math.pi if target_angle > 0 else target_angle + math.pi
                target_linear_speed = -target_linear_speed
            
            
            target_angular_speed = target_linear_speed / self.wheel_radius  # omega = v/R

            steering_commands.append(target_angle)
            drive_commands.append(target_angular_speed) # Gửi tốc độ góc

        # Gửi lệnh xuống Controller
        msg_steer = Float64MultiArray()
        msg_steer.data = steering_commands 
        self.pub_steering.publish(msg_steer)

        msg_drive = Float64MultiArray()
        msg_drive.data = drive_commands   
        self.pub_drive.publish(msg_drive)

    def update_odometry(self):
        # Tính thời gian trôi qua (dt)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # --- TÍNH TOÁN VỊ TRÍ (DEAD RECKONING) ---
        # dx, dy này là quãng đường xe đi được (m)
        delta_x = (self.current_vx * math.cos(self.theta) - self.current_vy * math.sin(self.theta)) * dt  # Nếu chỉ đi theo x thì sin(theta) = 0
        delta_y = (self.current_vx * math.sin(self.theta) + self.current_vy * math.cos(self.theta)) * dt  # Cũng như trên đi theo y thì cos = 0
        delta_th = self.current_omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th

        # --- PUBLISH ODOMETRY ---
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint" 

        # Set vị trí
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Chuyển Euler (theta) sang Quaternion
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)

        # --- PUBLISH TF ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint" 
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()