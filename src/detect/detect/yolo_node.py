import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')

        self.model = YOLO('src/detect/resource/best.pt')  
        self.model.to("cpu")
        self.get_logger().info("YOLO model loaded")

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/my_full_camera/image_raw',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Image,
            '/yolo/image',
            10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame, verbose=False)
        r = results[0]

    # ==== CHECK DETECTION ====
        if r.boxes is not None and len(r.boxes) > 0:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls_id]

                self.get_logger().info(
                    f"Detected: {class_name}, confidence={conf:.2f}"
                )

        annotated = results[0].plot()

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.pub.publish(out_msg)

        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()