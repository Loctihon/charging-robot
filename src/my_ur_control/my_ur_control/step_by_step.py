import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math

# Import file IK fen vừa sửa xong
from my_ur_control import ur10_inverse_kin 

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')
        
        self.VERTICAL_MODE_ROLL = 1.57 
        self.WRIST_1_OFFSET = -0.07

        self.input_sub = self.create_subscription(Point, '/goal_cmd', self.goal_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.current_joints = [0.0]*6
        
        self.get_logger().info(">>> READY TO PLUG: Vertical Mode Activated <<<")

    def joint_state_callback(self, msg):
        ur_joint_names = ['ur10_shoulder_pan_joint', 'ur10_shoulder_lift_joint', 'ur10_elbow_joint',
                          'ur10_wrist_1_joint', 'ur10_wrist_2_joint', 'ur10_wrist_3_joint']
        try:
            current_pos = []
            for name in ur_joint_names:
                if name in msg.name:
                    current_pos.append(msg.position[msg.name.index(name)])
            if len(current_pos) == 6: self.current_joints = current_pos
        except ValueError: pass

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

    def get_target_rotation_matrix(self, tx, ty):
        # 1. TRỤC Z: Cắm thẳng xuống đất (BẮT BUỘC)
        vec_z = np.array([0.0, 0.0, -1.0]) 

        # 2. TRỤC X: Hướng thẳng vào trạm sạc
        vec_x = np.array([tx, ty, 0.0])
        if np.linalg.norm(vec_x) > 0: vec_x /= np.linalg.norm(vec_x)
        
        # 3. TRỤC Y: Vuông góc (Nằm ngang)
        vec_y = np.cross(vec_z, vec_x)
        
        R = np.eye(3)
        R[:, 0] = vec_x
        R[:, 1] = vec_y
        R[:, 2] = vec_z

        # 4. XOAY ĐỂ DỰNG CHÂN SẠC (QUAN TRỌNG NHẤT)
        # Xoay quanh trục Z của Flange (đang hướng xuống đất)
        # Việc này sẽ xoay Wrist 3 -> Dựng đứng 2 chân pins lên.
        c, s = math.cos(self.VERTICAL_MODE_ROLL), math.sin(self.VERTICAL_MODE_ROLL)
        R_fix = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        
        return R @ R_fix

    def find_best_solution(self, x, y, z):
        R_target = self.get_target_rotation_matrix(x, y)
        T = np.eye(4); T[:3,:3]=R_target; T[0,3]=x; T[1,3]=y; T[2,3]=z

        sols = ur10_inverse_kin.inverse_kinematics(T)
        candidates = []

        for sol in sols:
            if sol is None: continue
            adj_sol = list(sol)
            
            # Bù Wrist 1 để Flange phẳng
            adj_sol[3] = adj_sol[3] + self.WRIST_1_OFFSET
            
            # Chỉ lấy nghiệm Vai Nâng (Shoulder Lift < 0)
            if adj_sol[1] > 0.0: continue
            
            # Chỉ lấy nghiệm Khuỷu Gập Lên (Elbow > 0) để tránh va chạm xe
            if adj_sol[2] < 0.0: continue

            # Chuẩn hóa góc về -pi đến pi
            adj_sol = [self.normalize_angle(a) for a in adj_sol]

            diff = sum([abs(s-c) for s,c in zip(adj_sol, self.current_joints)])
            candidates.append((diff, adj_sol))

        if not candidates: return None
        candidates.sort(key=lambda x: x[0])
        return candidates[0][1]

    def goal_callback(self, msg):
        self.get_logger().info(f"Đang tính toán cắm vào: x={msg.x}, y={msg.y}, z={msg.z}")
        
        # Giới hạn tầm với 1.3m
        if math.sqrt(msg.x**2 + msg.y**2 + msg.z**2) > 1.3:
            self.get_logger().error("XA QUÁ! Lùi xe lại gần chút đi fen.")
            return

        best_sol = self.find_best_solution(msg.x, msg.y, msg.z)
        if best_sol:
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ['ur10_shoulder_pan_joint', 'ur10_shoulder_lift_joint', 'ur10_elbow_joint',
                                    'ur10_wrist_1_joint', 'ur10_wrist_2_joint', 'ur10_wrist_3_joint']
            point = JointTrajectoryPoint()
            point.positions = best_sol
            point.time_from_start.sec = 4
            traj_msg.points.append(point)
            self.traj_pub.publish(traj_msg)
            self.get_logger().info("Đã gửi lệnh. Chân sạc sẽ DỰNG ĐỨNG.")
        else:
            self.get_logger().error("Không tìm thấy nghiệm. Kiểm tra lại tọa độ Z.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()