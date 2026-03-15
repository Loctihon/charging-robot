import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time

class AutoChargerSwerveNode(Node):
    def __init__(self):
        super().__init__('auto_charger_swerve_node')

        self.MARKER_SIZE = 0.0503  

        self.camera_matrix = np.array([[1513.75, 0, 640], 
                                       [0, 1513.75, 360], 
                                       [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros((5, 1))
        self.MAGIC_OFFSET_Y = -0.16  

        # KHOẢNG CÁCH DỪNG
        self.TARGET_DISTANCE_1 = 1.2
        self.TARGET_DISTANCE_2 = 0.35 


        self.JOINT_NAMES = [
            'ur10_shoulder_pan_joint', 'ur10_shoulder_lift_joint', 'ur10_elbow_joint',
            'ur10_wrist_1_joint', 'ur10_wrist_2_joint', 'ur10_wrist_3_joint'
        ]


        self.ARM_POSITIONS_DEG = [0.0, -183.5, -86.5, 0.0, 90.0, 0.0]

        # Topic
        self.camera_topic = '/base_camera/base_camera_sensor/image_raw'
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        self.sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.br = CvBridge()

        self.state = "SEARCHING" 
        self.arm_moved = False
        self.get_logger().info("Swerve Auto Charge Node (Quick Fix) đã khởi động!")

    def image_callback(self, data):
        try:
            frame = self.br.imgmsg_to_cv2(data, "bgr8")
        except:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        params = aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)

        distance = 999.0
        lateral_error = 0.0 
        angle_deg = 0.0
        angle_rad = 0.0
        detected = False

        if ids is not None:
            detected = True
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs)
            
            # Tọa độ từ Camera
            cam_x = tvecs[0][0][0] # Lệch trái phải
            cam_z = tvecs[0][0][2] # Khoảng cách xa gần (Depth)

            distance = math.sqrt(cam_x**2 + cam_z**2)

            lateral_error = cam_x + self.MAGIC_OFFSET_Y
            
            angle_rad = math.atan2(cam_x, cam_z)
            angle_deg = math.degrees(angle_rad)

            # Hiển thị HUD
            cv2.putText(frame, f"State: {self.state}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.putText(frame, f"Dist: {distance:.2f}m | Err: {lateral_error:.2f}m", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cmd = Twist()

        if self.state == "SEARCHING":
            if detected:
                self.get_logger().info("--> Tìm thấy! Bắt đầu tiếp cận.")
                self.state = "APPROACH"
            else:
                cmd.angular.z = 0.3 # Quay tìm kiếm

        elif self.state == "APPROACH":
            # Giai đoạn 1: Đi thẳng tới (Vừa đi vừa chỉnh hướng)
            if not detected:
                cmd.linear.x = 0.0
            else:
                cmd.angular.z = -1.0 * angle_rad # Giữ hướng nhìn thẳng vào ArUco
                
                if distance > self.TARGET_DISTANCE_1:
                    cmd.linear.x = 0.25 
                else:
                    self.stop_robot()
                    self.get_logger().info("--> Đã đến điểm 1m. Dừng xe. Múa tay.")
                    self.state = "ARM_READY"

        elif self.state == "ARM_READY":
            if not self.arm_moved:
                self.move_arm_to_charging_pose()
                self.arm_moved = True
                time.sleep(5.0) # Chờ 5 giây cho tay di chuyển xong
            
            self.state = "ALIGN"
            self.get_logger().info("--> Tay đã xong. Bắt đầu trượt ngang (ALIGN).")

        elif self.state == "ALIGN":
            # Giai đoạn 2: TRƯỢT NGANG (CRAB WALK)
            if not detected:
                self.stop_robot()
            else:
                cmd.angular.z = 0.0 

                raw_sway_speed = -0.8 * lateral_error 
                
                cmd.linear.y = np.clip(raw_sway_speed, -0.15, 0.15)
                
                cmd.linear.x = 0.0

                # Điều kiện xong: Lệch dưới 1.5cm
                if abs(lateral_error) < 0.015:
                    self.stop_robot()
                    self.get_logger().info("--> Đã thẳng hàng! Bắt đầu cắm vào (DOCKING).")
                    self.state = "DOCKING"
                else:
                    # In ra để debug
                    pass

        elif self.state == "DOCKING":
            if not detected:
                # Mất dấu (do gần quá) Đi thật chậm
                cmd.linear.x = 0.05
                cmd.angular.z = 0.0 
            else:
                if distance > self.TARGET_DISTANCE_2:
                    cmd.linear.x = 0.08 
                    cmd.linear.y = np.clip(-1.0 * lateral_error, -0.1, 0.1) # Vẫn chỉnh ngang nhẹ
                    cmd.angular.z = 0.0  # Vẫn khóa góc
                else:
                    self.stop_robot()
                    self.state = "FINISHED"
                    self.get_logger().info("✅ KẾT THÚC! ĐÃ CẮM SẠC.")

        elif self.state == "FINISHED":
            self.stop_robot()

        self.cmd_vel_pub.publish(cmd)
        cv2.imshow("SWERVE CHARGER QUICK FIX", frame)
        cv2.waitKey(1)

    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def move_arm_to_charging_pose(self):
        msg = JointTrajectory()
        msg.joint_names = self.JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [math.radians(angle) for angle in self.ARM_POSITIONS_DEG]
        
        point.time_from_start.sec = 5
        msg.points.append(point)
        self.arm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoChargerSwerveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()