import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math

class CameraTester(Node):
    def __init__(self):
        super().__init__('camera_tester_node')
        
        self.camera_topic = '/base_camera/base_camera_sensor/image_raw'
        
        self.subscription = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10)
        
        self.br = CvBridge()
        self.get_logger().info(f"👀 Đang soi camera: {self.camera_topic}")

        self.camera_matrix = np.array([[623.54, 0, 360],
                                       [0, 623.54, 180],
                                       [0, 0, 1]], dtype=float)
        
        self.dist_coeffs = np.zeros((5, 1))

        self.marker_size = 0.0503 

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.aruco_params = aruco.DetectorParameters()

    def image_callback(self, data):
        try:
            frame = self.br.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            # --- TÍNH TOÁN 3D ---
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            # tvecs[0][0] chứa [x, y, z]
            # x: Lệch trái/phải
            # y: Lệch lên/xuống
            # z: Khoảng cách xa/gần (Depth)
            x = tvecs[0][0][0]
            z = tvecs[0][0][2]

            distance = math.sqrt(x**2 + z**2)
            
            # Tính góc lệch (atan2 trả về radian -> đổi sang độ)
            angle_rad = math.atan2(x, z)
            angle_deg = math.degrees(angle_rad)

            # Hiển thị thông tin lên màn hình
            info_text = f"Dist: {distance:.2f}m | Angle: {angle_deg:.1f} deg"
            cv2.putText(frame, info_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.8, (0, 0, 255), 2)
            
            # In ra terminal để debug
            print(f"Target ID={ids[0][0]}: Cách {z:.2f}m, Lệch {x:.2f}m, Góc {angle_deg:.1f} độ")

        cv2.imshow("TEST CAMERA - ARUCO 3D", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()