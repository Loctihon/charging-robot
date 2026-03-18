import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import Image

import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def order_points(pts):
    """Order: top-left, top-right, bottom-right, bottom-left."""
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


class PnPNode(Node):

    def __init__(self):
        super().__init__('pnp_node')

        # ===== IMAGE SUB =====
        self.latest_frame = None
        self.sub = self.create_subscription(
            Image,
            '/base_camera/base_camera_sensor/image_raw',
            self.image_callback,
            10)

        self.w = 1280
        self.h = 720

        self.timer = self.create_timer(1 / 30, self.process_frame)

        # ===== PUBLISH =====
        self.pub = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.image_pub = self.create_publisher(Image, '/pnp_debug/image', 10)
        self.mask_pub = self.create_publisher(Image, '/pnp_debug/mask', 10)
        self.tvec_pub = self.create_publisher(Vector3, '/pnp/tvec', 10)
        self.rvec_pub = self.create_publisher(Vector3, '/pnp/rvec', 10)

        # ===== COLOR =====
        # Wider range to catch bronze / dark-yellow plate
        self.yellow_lower = np.array([10, 50, 30])
        self.yellow_upper = np.array([40, 255, 200])

        self.kernel_open = np.ones((3, 3), np.uint8)
        self.kernel_close = np.ones((7, 7), np.uint8)

        # ===== CAMERA =====
        # FOV ~60° horizontal is a reasonable webcam default.
        # Adjust this if you know the real FOV of your camera.
        HFOV_DEG = 60.0
        FX = self.w / (2 * np.tan(np.radians(HFOV_DEG / 2)))
        FY = FX
        CX = self.w / 2
        CY = self.h / 2

        self.K = np.array([
            [FX, 0, CX],
            [0, FY, CY],
            [0,  0,  1]
        ], dtype=np.float64)

        self.dist = np.zeros((4, 1), dtype=np.float64)

        self.get_logger().info(
            f"Camera matrix: fx={FX:.1f}  fy={FY:.1f}  "
            f"cx={CX:.1f}  cy={CY:.1f}")

        # ===== OBJECT (plate 90 mm x 117.5 mm) =====
        # Points ordered same as order_points: TL, TR, BR, BL
        hw = 0.045      # half-width
        hh = 0.05875    # half-height
        self.object_points = np.array([
            [-hw,  hh, 0],   # top-left
            [ hw,  hh, 0],   # top-right
            [ hw, -hh, 0],   # bottom-right
            [-hw, -hh, 0],   # bottom-left
        ], dtype=np.float64)

        # ===== SMOOTH =====
        self.alpha = 0.3  # lower = more responsive, less drift
        self.rot_smooth = None   # Rotation object
        self.tvec_smooth = None
        self.prev_rvec = None
        self.prev_tvec = None
        self.max_jump = 0.15  # max translation jump (m) before reset

    def _imgmsg_to_cv2(self, msg):
        dtype = np.uint8
        if msg.encoding in ('rgb8', 'bgr8'):
            channels = 3
        elif msg.encoding in ('rgba8', 'bgra8'):
            channels = 4
        elif msg.encoding == 'mono8':
            channels = 1
        else:
            channels = 3
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def _cv2_to_imgmsg(self, cv_image, encoding='bgr8'):
        msg = Image()
        msg.height, msg.width = cv_image.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        if encoding == 'mono8':
            msg.step = msg.width
        else:
            msg.step = msg.width * 3
        msg.data = cv_image.tobytes()
        return msg

    def image_callback(self, msg):
        frame = self._imgmsg_to_cv2(msg)
        self.latest_frame = frame
        self.h, self.w = frame.shape[:2]

    def process_frame(self):
        if self.latest_frame is None:
            return
        frame = self.latest_frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        # Open first to kill noise, then close to fill gaps
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False

        if contours:
            biggest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest)

            if area > 500:
                # Use minAreaRect → always gives 4 corners, more stable
                rect = cv2.minAreaRect(biggest)
                box = cv2.boxPoints(rect)
                image_points = order_points(box).astype(np.float64)

                # Draw detected rectangle
                cv2.drawContours(
                    frame, [image_points.astype(int)], -1, (0, 255, 255), 2)

                # Use previous solution as initial guess for stability
                use_extrinsic = (self.prev_rvec is not None)
                flags = cv2.SOLVEPNP_IPPE_SQUARE

                if use_extrinsic:
                    success, rvec, tvec = cv2.solvePnP(
                        self.object_points,
                        image_points,
                        self.K,
                        self.dist,
                        rvec=self.prev_rvec.copy(),
                        tvec=self.prev_tvec.copy(),
                        useExtrinsicGuess=True,
                        flags=cv2.SOLVEPNP_ITERATIVE,
                    )
                else:
                    success, rvec, tvec = cv2.solvePnP(
                        self.object_points,
                        image_points,
                        self.K,
                        self.dist,
                        flags=flags,
                    )

                if success:
                    # Check reprojection error — reject bad solutions
                    reproj, _ = cv2.projectPoints(
                        self.object_points, rvec, tvec, self.K, self.dist)
                    err = np.mean(np.linalg.norm(
                        reproj.reshape(-1, 2) - image_points, axis=1))
                    if err > 10.0:  # pixels — skip junk
                        detected = False
                    else:
                        self.prev_rvec = rvec.copy()
                        self.prev_tvec = tvec.copy()

                        rot_cur = Rotation.from_rotvec(rvec.flatten())

                        # If view moved a lot, reset smoother instead of dragging
                        big_jump = (self.tvec_smooth is not None and
                                    np.linalg.norm(tvec.flatten() - self.tvec_smooth.flatten())
                                    > self.max_jump)

                        if self.rot_smooth is None or big_jump:
                            self.rot_smooth = rot_cur
                            self.tvec_smooth = tvec.copy()
                        else:
                            a = self.alpha
                            # SLERP for rotation — mathematically correct
                            slerp = Slerp([0, 1], Rotation.concatenate(
                                [self.rot_smooth, rot_cur]))
                            self.rot_smooth = slerp(1 - a)
                            # Linear interp for translation is fine
                            self.tvec_smooth = a * self.tvec_smooth + (1 - a) * tvec

                        # Get smoothed rvec for drawing
                        rvec_s = self.rot_smooth.as_rotvec().reshape(3, 1)
                        tvec_s = self.tvec_smooth

                        # ---- Publish full 6-DOF pose ----
                        quat = self.rot_smooth.as_quat()  # [x,y,z,w]

                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = "camera"

                        pose_msg.pose.position.x = float(tvec_s[0])
                        pose_msg.pose.position.y = float(tvec_s[1])
                        pose_msg.pose.position.z = float(tvec_s[2])

                        pose_msg.pose.orientation.x = float(quat[0])
                        pose_msg.pose.orientation.y = float(quat[1])
                        pose_msg.pose.orientation.z = float(quat[2])
                        pose_msg.pose.orientation.w = float(quat[3])

                        self.pub.publish(pose_msg)

                        # ---- Publish tvec & rvec ----
                        t = tvec_s.flatten()
                        r = rvec_s.flatten()
                        tvec_msg = Vector3(x=float(t[0]), y=float(t[1]), z=float(t[2]))
                        rvec_msg = Vector3(x=float(r[0]), y=float(r[1]), z=float(r[2]))
                        self.tvec_pub.publish(tvec_msg)
                        self.rvec_pub.publish(rvec_msg)

                        # ---- Draw 3D axes ----
                        axis_len = 0.05
                        axis = np.float64([
                            [0, 0, 0],
                            [axis_len, 0, 0],
                            [0, axis_len, 0],
                            [0, 0, axis_len],
                        ])
                        imgpts, _ = cv2.projectPoints(
                            axis, rvec_s, tvec_s, self.K, self.dist)
                        imgpts = imgpts.reshape(-1, 2).astype(int)

                        o, x, y, z = imgpts
                        cv2.line(frame, tuple(o), tuple(x), (0, 0, 255), 3)
                        cv2.line(frame, tuple(o), tuple(y), (0, 255, 0), 3)
                        cv2.line(frame, tuple(o), tuple(z), (255, 0, 0), 3)

                        # Info overlay
                        t = tvec_s.flatten()
                        cv2.putText(
                            frame,
                            f"x={t[0]:.3f} y={t[1]:.3f} z={t[2]:.3f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0), 2)

                        detected = True

        if not detected:
            cv2.putText(
                frame, "NO DETECTION", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Always publish debug images so topics appear in `ros2 topic list`
        self.image_pub.publish(self._cv2_to_imgmsg(frame, encoding='bgr8'))
        self.mask_pub.publish(self._cv2_to_imgmsg(mask, encoding='mono8'))


def main(args=None):
    rclpy.init(args=args)
    node = PnPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()