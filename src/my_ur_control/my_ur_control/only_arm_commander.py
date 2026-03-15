import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from math import cos, sin, pi 

# Import file toán IK
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

        # ===> KHỐI 5: ĐỒNG HỒ BÁO CÁO (FK TIMER) <===
        # Cứ 1 giây sẽ tính toán và in tọa độ thực tế 1 lần
        self.timer = self.create_timer(1.0, self.print_real_position)

        self.get_logger().info("Arm Commander (No Prefix + FK) đã sẵn sàng!")

    # --- HÀM TÍNH TOÁN VỊ TRÍ THỰC TẾ (Forward Kinematics) ---
    def calculate_fk(self, joints):
        # Bộ tham số DH (Lấy từ file ur10_inverse_kin.py của bạn)
        d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]
        a = [0, -0.612, -0.5723, 0, 0, 0]
        alph = [pi/2, 0, 0, pi/2, -pi/2, 0]
        
        T = np.eye(4) # Ma trận đơn vị

        # Nhân 6 ma trận biến đổi lại với nhau
        for i in range(6):
            th = joints[i]
            ct = cos(th)
            st = sin(th)
            ca = cos(alph[i])
            sa = sin(alph[i])
            
            # Ma trận biến đổi DH cơ bản
            A_i = np.array([
                [ct, -st*ca, st*sa, a[i]*ct],
                [st, ct*ca, -ct*sa, a[i]*st],
                [0, sa, ca, d[i]],
                [0, 0, 0, 1]
            ])
            
            T = np.dot(T, A_i)
        
        return T[0,3], T[1,3], T[2,3]

    def print_real_position(self):
        if self.current_joints:
            real_x, real_y, real_z = self.calculate_fk(self.current_joints)
            self.get_logger().info(f"TỌA ĐỘ THỰC TẾ (FK): X={real_x:.3f}, Y={real_y:.3f}, Z={real_z:.3f}")

    def joint_state_callback(self, msg):
        if not hasattr(self, 'checked_names'):
            self.get_logger().warn(f"Robot đang gửi về các khớp: {msg.name}")
            self.checked_names = True

        ur_joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        try:
            current_pos = []
            for name in ur_joint_names:
                if name in msg.name:
                    index = msg.name.index(name)
                    current_pos.append(msg.position[index])
                else:
                    return 
            self.current_joints = current_pos
        except ValueError:
            pass

    def goal_callback(self, msg):
        target_x, target_y, target_z = msg.x, msg.y, msg.z
        
        self.get_logger().info(f"--- NHẬN LỆNH MỚI: X={target_x}, Y={target_y}, Z={target_z} ---")

        T = np.eye(4)
        # Lưu ý: Kiểm tra dấu trừ nếu robot bị ngược
        T[0,3] = target_x
        T[1,3] = target_y
        T[2,3] = target_z
        
        solutions = ur10_inverse_kin.inverse_kinematics(T) 
        best_sol = self.find_closest_solution(solutions, self.current_joints)

        if best_sol:
            self.get_logger().info(f"-> Đã tính ra nghiệm IK. Đang gửi lệnh...")
            
            # Check an toàn quay quá 180 độ
            max_jump = max([abs(s - c) for s, c in zip(best_sol, self.current_joints)])
            if max_jump > 3.14: 
                self.get_logger().error("CẢNH BÁO: Robot quay quá lớn (>180 độ)!")
            
            self.send_command(best_sol)
        else:
            self.get_logger().error("KHÔNG TÌM THẤY NGHIỆM IK!")

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
        # Tên khớp gửi đi KHÔNG CÓ ur10_
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 4 
        msg.points.append(point)
        self.traj_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()