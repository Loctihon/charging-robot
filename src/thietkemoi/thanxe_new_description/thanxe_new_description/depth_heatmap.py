import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthHeatmap(Node):

    def __init__(self):
        super().__init__('depth_heatmap')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/base_camera/base_camera_sensor/depth/image_raw',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            Image,
            '/base_camera/depth_heatmap',
            10
        )

        self.get_logger().info('Depth heatmap node started.')

    def callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Convert to float32 if needed
        depth = np.float32(depth)

        # Replace inf/nan with 0
        depth = np.where(np.isfinite(depth), depth, 0.0)

        # Normalize to 0-255
        d_min = np.min(depth[depth > 0]) if np.any(depth > 0) else 0.0
        d_max = np.max(depth)

        if d_max - d_min > 0:
            normalized = (depth - d_min) / (d_max - d_min)
        else:
            normalized = np.zeros_like(depth)

        # Convert to uint8 (0 = near, 255 = far)
        gray = (normalized * 255).astype(np.uint8)

        # Apply colormap: COLORMAP_JET maps 0->blue, 255->red
        # We flip so that near=red, far=blue
        gray_flipped = 255 - gray
        heatmap = cv2.applyColorMap(gray_flipped, cv2.COLORMAP_JET)

        # Set invalid depth (0) to black
        heatmap[depth <= 0] = [0, 0, 0]

        # Publish
        heatmap_msg = self.bridge.cv2_to_imgmsg(heatmap, encoding='bgr8')
        heatmap_msg.header = msg.header
        self.pub.publish(heatmap_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthHeatmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
