import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraRecorder(Node):

    def __init__(self):
        super().__init__('camera_recorder')

        # Đảm bảo topic này trùng khớp với topic camera trong Gazebo
        self.topic = '/base_camera/base_camera_sensor/image_raw'
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            self.topic,
            self.callback,
            10
        )

        self.video_writer = None
        self.get_logger().info(f"Đang chờ luồng hình ảnh từ topic: {self.topic} ...")

    def callback(self, msg):
        # Chuyển đổi format ảnh ROS sang OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Khởi tạo VideoWriter khi nhận được frame đầu tiên
        if self.video_writer is None:
            h, w, _ = frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            
            # File sẽ lưu tại thư mục bạn đang đứng gõ lệnh (thường là ~/new_design_ws)
            self.video_writer = cv2.VideoWriter(
                'camera_record.mp4',
                fourcc,
                30.0, # Tốc độ khung hình (FPS)
                (w, h)
            )
            self.get_logger().info("✅ Đã nhận được khung hình đầu tiên! Đang quay video...")

        # Ghi frame vào file
        self.video_writer.write(frame)

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info("💾 Đã lưu thành công file video 'camera_record.mp4'!")
        super().destroy_node()

def main(args=None): # ĐÃ SỬA: Thêm args=None
    rclpy.init(args=args) # ĐÃ SỬA: Truyền args vào init

    node = CameraRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Đã nhận lệnh dừng (Ctrl+C). Đang đóng gói video...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()