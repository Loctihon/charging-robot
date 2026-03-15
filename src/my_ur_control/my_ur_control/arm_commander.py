import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math

from my_ur_control import ur10_inverse_kin 

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')

        # --- KHỐI 1: TAI NGHE (Nhận lệnh X,Y,Z) ---
        self.input_sub = self.create_subscription(
            Point, 
            '/goal_cmd', 
            self.goal_callback, 
            10
        )

        # --- KHỐI 2: MẮT THẦN (Biết robot đang ở đâu) ---
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        self.current_joints = [0.0] * 6 # Lưu tạm vị trí hiện tại

        # --- KHỐI 4: CÁI MIỆNG (Gửi lệnh đi) ---
        self.traj_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )

        self.get_logger().info("Arm Commander đã sẵn sàng! Gửi Point vào /goal_cmd đi!")

    def joint_state_callback(self, msg):
        if not hasattr(self, 'checked_names'):
            self.get_logger().warn(f"Robot đang gửi về các khớp: {msg.name}")
            self.checked_names = True

        ur_joint_names = [
            'ur10_shoulder_pan_joint', 'ur10_shoulder_lift_joint', 'ur10_elbow_joint',
            'ur10_wrist_1_joint', 'ur10_wrist_2_joint', 'ur10_wrist_3_joint'
        ]
        try:
            current_pos = []
            for name in ur_joint_names:
                if name in msg.name:
                    index = msg.name.index(name)
                    current_pos.append(msg.position[index])
                else:
                    return # Không cập nhật thêm
            
            self.current_joints = current_pos
            
        except ValueError:
            pass

    # --- HÀM MỚI: ĐỘNG HỌC THUẬN (FK) ĐỂ KIỂM TRA ---
    def calculate_fk(self, joints):
        # Thông số DH của UR10 (Khớp với file IK của bạn)
        d = [0.1273, 0, 0, 0.1639, 0.1157, 0.0922]
        a = [0, -0.612, -0.5723, 0, 0, 0]
        alph = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]
        
        # Ma trận biến đổi tổng
        T = np.eye(4)
        
        for i in range(6):
            theta = joints[i]
            # Tạo ma trận biến đổi cho khớp thứ i
            ct = math.cos(theta)
            st = math.sin(theta)
            ca = math.cos(alph[i])
            sa = math.sin(alph[i])
            
            # Ma trận DH tiêu chuẩn (giống logic file IK của bạn)
            # T_i = Rot_z * Trans_z * Trans_x * Rot_x
            # Lưu ý: File IK của bạn dùng thứ tự: T_d * Rzt * T_a * Rxa
            # Nên ta viết tường minh theo đúng logic đó:
            
            # 1. Rot_z(theta)
            Rzt = np.array([
                [ct, -st, 0, 0],
                [st,  ct, 0, 0],
                [0,    0, 1, 0],
                [0,    0, 0, 1]
            ])
            # 2. Trans_z(d)
            Td = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, d[i]],
                [0, 0, 0, 1]
            ])
            # 3. Trans_x(a)
            Ta = np.array([
                [1, 0, 0, a[i]],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            # 4. Rot_x(alpha)
            Rxa = np.array([
                [1, 0,   0, 0],
                [0, ca, -sa, 0],
                [0, sa,  ca, 0],
                [0, 0,   0, 1]
            ])
            
            # Nhân ma trận: A_i = Td * Rzt * Ta * Rxa
            A_i = Td @ Rzt @ Ta @ Rxa
            
            # Nhân vào ma trận tổng
            T = T @ A_i
            
        # Trả về tọa độ X, Y, Z
        return T[0,3], T[1,3], T[2,3]

    # --- KHỐI 3: BỘ NÃO (Xử lý chính) ---
    def goal_callback(self, msg):
        target_x, target_y, target_z = msg.x, msg.y, msg.z
        self.get_logger().info(f"--- NHẬN LỆNH MỚI ---")
        self.get_logger().info(f"Mục tiêu (Target): x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")
        
        T = np.eye(4)
        T[0,3] = target_x
        T[1,3] = target_y
        T[2,3] = target_z
        
        # 1. Giải IK
        solutions = ur10_inverse_kin.inverse_kinematics(T) 
        best_sol = self.find_closest_solution(solutions, self.current_joints)

        if best_sol:
            # 2. KIỂM TRA NGƯỢC LẠI BẰNG FK (Bổ sung mới)
            # Tính xem với góc best_sol này, robot thực sự sẽ đi đến đâu
            fk_x, fk_y, fk_z = self.calculate_fk(best_sol)
            
            self.get_logger().info("--- KẾT QUẢ KIỂM TRA (Verify IK) ---")
            self.get_logger().warn(f"Nghiệm IK (Góc khớp): {[round(x,2) for x in best_sol]}")
            self.get_logger().info(f"Tọa độ thực tế sẽ đạt tới (FK): x={fk_x:.3f}, y={fk_y:.3f}, z={fk_z:.3f}")
            
            # Tính sai số
            error = math.sqrt((target_x-fk_x)**2 + (target_y-fk_y)**2 + (target_z-fk_z)**2)
            self.get_logger().info(f"Sai số khoảng cách (Error): {error:.5f} m")

            max_jump = max([abs(s - c) for s, c in zip(best_sol, self.current_joints)])
            
            if max_jump > 3.14: 
                self.get_logger().error("CẢNH BÁO: Robot đang định quay quá 180 độ!")
            
            self.send_command(best_sol)
        else:
            self.get_logger().error("Không tìm thấy nghiệm IK nào phù hợp!")

    def find_closest_solution(self, solutions, current_joints):
        best_sol = None
        min_diff = float('inf')
        
        for sol in solutions:
            diff = sum([abs(s - c) for s, c in zip(sol, current_joints)])
            if diff < min_diff:
                min_diff = diff
                best_sol = sol
        return best_sol

    def send_command(self, joint_angles):
        msg = JointTrajectory()
        msg.joint_names = [
            'ur10_shoulder_pan_joint', 'ur10_shoulder_lift_joint', 'ur10_elbow_joint',
            'ur10_wrist_1_joint', 'ur10_wrist_2_joint', 'ur10_wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 4 
        
        msg.points.append(point)
        self.traj_pub.publish(msg)
        self.get_logger().info("Đã gửi lệnh xuống Controller!")

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()