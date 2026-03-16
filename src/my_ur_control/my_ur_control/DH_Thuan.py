import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math
import sys

d = [0.1273, 0, 0, 0.1639, 0.1157, 0.0922]
a = [0, -0.612, -0.5723, 0, 0, 0]
alph = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]

class FKCommander(Node):
    def __init__(self):
        super().__init__('fk_commander')
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.get_logger().info(">>>(FORWARD KINEMATICS) <<<")
        self.get_logger().info("Đảm bảo theta2,3,4 phải bằng -270")
        self.get_logger().info("Nhập 6 góc (độ) cách nhau bằng dấu cách. Ví dụ: 0 -90 -90 -90 90 0")

    def send_joints(self, joints_deg):
        # Chuyển độ sang radian
        joints_rad = [math.radians(j) for j in joints_deg]
        
        # Gửi lệnh tới Gazebo
        msg = JointTrajectory()
        msg.joint_names = ['ur10_shoulder_pan_joint', 'ur10_shoulder_lift_joint', 'ur10_elbow_joint',
                           'ur10_wrist_1_joint', 'ur10_wrist_2_joint', 'ur10_wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = joints_rad
        point.time_from_start.sec = 2
        msg.points.append(point)
        self.traj_pub.publish(msg)
        
        # Tính toán vị trí Flange (FK)
        T = self.calculate_fk(joints_rad)
        self.print_pose(T, joints_deg)

    def calculate_fk(self, q):
        T = np.eye(4)
        for i in range(6):
            ct = math.cos(q[i])
            st = math.sin(q[i])
            ca = math.cos(alph[i])
            sa = math.sin(alph[i])
            
            # Ma trận chuyển đổi DH
            A = np.array([
                [ct, -st*ca, st*sa,  a[i]*ct],
                [st,  ct*ca, -ct*sa, a[i]*st],
                [0,   sa,     ca,     d[i]],
                [0,   0,      0,      1]
            ])
            T = T @ A
        return T

    def rotation_matrix_to_rpy(self, R):
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.degrees([x, y, z])

    def print_pose(self, T, joints):
        x, y, z = T[0,3], T[1,3], T[2,3]
        r, p, yw = self.rotation_matrix_to_rpy(T[:3,:3])
        
        print("\n" + "="*40)
        print(f"GÓC KHỚP (ĐỘ): {joints}")
        print("-" * 40)
        print(f"VỊ TRÍ FLANGE (Mặt bích):")
        print(f"   X = {x:.5f}")
        print(f"   Y = {y:.5f}")
        print(f"   Z = {z:.5f}")
        print("-" * 40)
        print(f"GÓC QUAY FLANGE (RPY - Độ):")
        print(f"   Roll  = {r:.2f}")
        print(f"   Pitch = {p:.2f}")
        print(f"   Yaw   = {yw:.2f}")
        print("="*40 + "\n")

def main():
    rclpy.init()
    node = FKCommander()
    
    # Góc mặc định ban đầu (Tư thế gập gọn)
    current_joints = [0, -90, 0, -90, 0, 0] 
    
    try:
        while rclpy.ok():
            try:
                user_input = input("Nhập 6 góc (ví dụ: 0 -90 -90 -90 90 90): ")
                
                parts = user_input.split()
                if len(parts) != 6:
                    print("Vui lòng nhập đúng 6 số!")
                    continue
                    
                target_joints = [float(x) for x in parts]
                node.send_joints(target_joints)
                
            except ValueError:
                print("Lỗi nhập liệu! Chỉ nhập số.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()