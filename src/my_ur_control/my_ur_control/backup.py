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

    # --- KHỐI 3: BỘ NÃO (Xử lý chính) ---
    def goal_callback(self, msg):
        target_x, target_y, target_z = msg.x, msg.y, msg.z
        self.get_logger().info(f"Nhận lệnh đến: {target_x}, {target_y}, {target_z}")
        T = np.eye(4)
        T[0,3] = target_x
        T[1,3] = target_y
        T[2,3] = target_z
        solutions = ur10_inverse_kin.inverse_kinematics(T) 
        best_sol = self.find_closest_solution(solutions, self.current_joints)

        if best_sol:
            self.get_logger().warn(f"Vị trí hiện tại: {[round(x,2) for x in self.current_joints]}")
            self.get_logger().warn(f"Nghiệm IK tính ra: {[round(x,2) for x in best_sol]}")
            max_jump = max([abs(s - c) for s, c in zip(best_sol, self.current_joints)])
            self.get_logger().warn(f"Độ lệch lớn nhất: {max_jump:.2f} rad")

            if max_jump > 3.14: # Lớn hơn 180 độ
                self.get_logger().error("CẢNH BÁO: Robot đang định quay quá 180")
            self.send_command(best_sol)
        
        

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
        point.time_from_start.sec = 4 # Cho 4 giây để chạy tới đích 
        
        msg.points.append(point)
        self.traj_pub.publish(msg)
        self.get_logger().info("Đã gửi lệnh xuống Controller!")

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()