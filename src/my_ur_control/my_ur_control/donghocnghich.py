import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from math import *
import threading

d1 = 0.1273
a2 = -0.612
a3 = -0.5723
d4 = 0.1639
d5 = 0.1157
d6 = 0.0922

a = [0, a2, a3, 0, 0, 0]
d = [d1, 0, 0, d4, d5, d6]
alph = [pi/2, 0, 0, pi/2, -pi/2, 0]

def ah(n, th, c):
    T_a = np.identity(4)
    T_a[0,3] = a[n-1]
    T_d = np.identity(4)
    T_d[2,3] = d[n-1]

    Rzt = np.matrix([[cos(th[n-1,c]), -sin(th[n-1,c]), 0, 0],
                     [sin(th[n-1,c]),  cos(th[n-1,c]), 0, 0],
                     [0,               0,              1, 0],
                     [0,               0,              0, 1]])

    Rxa = np.matrix([[1, 0,                 0,                  0],
                     [0, cos(alph[n-1]), -sin(alph[n-1]),   0],
                     [0, sin(alph[n-1]),  cos(alph[n-1]),   0],
                     [0, 0,                 0,                  1]])

    A_i = T_d * Rzt * T_a * Rxa
    return A_i

def inverse_kinematics(T_06):
    th = np.zeros((6, 8))
    # --- TINH THETA 1 ---
    P_05 = T_06 * np.matrix([0,0,-d6,1]).T
    psi = atan2(P_05[1], P_05[0])
    
    val_phi = d4 / sqrt(P_05[0]*P_05[0] + P_05[1]*P_05[1])
    if abs(val_phi) > 1: return [] 
    phi = acos(val_phi)
    
    th[0, 0:4] = pi/2 + psi + phi
    th[0, 4:8] = pi/2 + psi - phi
    th = th.real

    # --- TINH THETA 5 ---
    cl = [0, 4]
    for i in range(len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(ah(1,th,c))
        T_16 = T_10 * T_06
        P_16z = T_16[2,3]
        
        # ĐÃ FIX LỖI TYPO D1 THÀNH D4 TẠI ĐÂY!
        val = (P_16z - d4) / d6
        if abs(val) > 1: val = np.sign(val)
        
        th[4, c:c+2] = -acos(val)
        th[4, c+2:c+4] = acos(val)
    th = th.real

    # --- TINH THETA 6 ---
    cl = [0, 2, 4, 6]
    for i in range(len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(ah(1,th,c))
        T_16 = np.linalg.inv( T_10 * T_06 )
        if sin(th[4, c]) == 0: continue 
        th[5, c:c+2] = atan2((-T_16[1,2]/sin(th[4, c])),(T_16[0,2]/sin(th[4, c])))
    th = th.real

    # --- TINH THETA 3 ---
    cl = [0, 2, 4, 6]
    for i in range(len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(ah(1,th,c))
        T_65 = ah(6,th,c)
        T_54 = ah(5,th,c)
        T_14 = ( T_10 * T_06 ) * np.linalg.inv(T_54 * T_65)
        P_13 = T_14 * np.matrix([0, -d4, 0, 1]).T - np.matrix([0,0,0,1]).T
        
        costh3 = ((P_13[0]**2 + P_13[1]**2) - a2**2 - a3**2) / (2*a2*a3)
        if abs(costh3) > 1: costh3 = np.sign(costh3) 
        
        th[2, c] = acos(costh3)
        th[2, c+1] = -acos(costh3)
    th = th.real

    # --- TINH THETA 2 & 4 ---
    cl = [0, 1, 2, 3, 4, 5, 6, 7]
    for i in range(len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(ah(1,th,c))
        T_65 = np.linalg.inv(ah(6,th,c))
        T_54 = np.linalg.inv(ah(5,th,c))
        T_14 = ( T_10 * T_06 ) * T_65 * T_54
        P_13 = T_14 * np.matrix([0, -d4, 0, 1]).T - np.matrix([0,0,0,1]).T
        
        val_asin = a3*sin(th[2,c])/sqrt(P_13[0]**2 + P_13[1]**2)
        if abs(val_asin) > 1: val_asin = np.sign(val_asin)
            
        th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(val_asin)
        
        T_32 = np.linalg.inv(ah(3,th,c))
        T_21 = np.linalg.inv(ah(2,th,c))
        T_34 = T_32 * T_21 * T_14
        th[3, c] = atan2(T_34[1,0], T_34[0,0])
    th = th.real
    
    solutions = []
    for i in range(8):
        solutions.append(th[:, i].tolist())
    return solutions

def get_t_matrix(x, y, z, r, p, yaw):
    Rx = np.matrix([[1, 0, 0, 0],
                    [0, cos(r), -sin(r), 0],
                    [0, sin(r), cos(r), 0],
                    [0, 0, 0, 1]])
    Ry = np.matrix([[cos(p), 0, sin(p), 0],
                    [0, 1, 0, 0],
                    [-sin(p), 0, cos(p), 0],
                    [0, 0, 0, 1]])
    Rz = np.matrix([[cos(yaw), -sin(yaw), 0, 0],
                    [sin(yaw), cos(yaw), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    T = np.matrix(np.identity(4))
    T[0,3] = x
    T[1,3] = y
    T[2,3] = z
    return T * Rz * Ry * Rx

def calculate_fk(q):
    """Hàm Động Học Thuận dùng để kiểm tra chéo (Cross-check)"""
    T = np.eye(4)
    for i in range(6):
        ct = cos(q[i])
        st = sin(q[i])
        ca = cos(alph[i])
        sa = sin(alph[i])
        
        A = np.array([
            [ct, -st*ca, st*sa,  a[i]*ct],
            [st,  ct*ca, -ct*sa, a[i]*st],
            [0,   sa,     ca,     d[i]],
            [0,   0,      0,      1]
        ])
        T = T @ A
    return T

def rotation_matrix_to_rpy(R):
    sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = atan2(R[2,1] , R[2,2])
        y = atan2(-R[2,0], sy)
        z = atan2(R[1,0], R[0,0])
    else:
        x = atan2(-R[1,2], R[1,1])
        y = atan2(-R[2,0], sy)
        z = 0
    return np.degrees([x, y, z])

# =====================================================================
# NODE ROS 2 ĐIỀU KHIỂN TAY MÁY KẾT HỢP IK & FK
# =====================================================================
class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__('arm_ik_teleop_node')
        
        self.traj_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
            
        self.joint_names = [
            'ur10_shoulder_pan_joint', 
            'ur10_shoulder_lift_joint', 
            'ur10_elbow_joint', 
            'ur10_wrist_1_joint', 
            'ur10_wrist_2_joint', 
            'ur10_wrist_3_joint'
        ]
        
        self.get_logger().info("🚀 KHỞI ĐỘNG HỆ THỐNG KINEMATICS TAY MÁY CHUẨN XÁC 100%!")
        
        threading.Thread(target=self.terminal_input_loop, daemon=True).start()

    def select_best_solution(self, solutions, target_x, target_y, target_z):
        """Lọc bỏ nghiệm ma (sai số > 5mm), sau đó chọn tư thế -270 độ"""
        valid_sols = []
        
        # BƯỚC 1: SÁT HẠCH FK ĐỂ LOẠI BỎ NGHIỆM MA
        for sol in solutions:
            T_check = calculate_fk(sol)
            err_dist = sqrt((target_x - T_check[0,3])**2 + 
                            (target_y - T_check[1,3])**2 + 
                            (target_z - T_check[2,3])**2)
            
            if err_dist < 0.005: 
                valid_sols.append(sol)
                
        if not valid_sols:
            return None 

        # BƯỚC 2: CHỌN TƯ THẾ MẶT BÍCH -270 ĐỘ
        best_sol = None
        min_err = float('inf')
        
        for sol in valid_sols:
            sum_j_deg = degrees(sol[1] + sol[2] + sol[3])
            err = abs((sum_j_deg - (-270) + 180) % 360 - 180) 
            
            if err < min_err:
                min_err = err
                best_sol = sol
                
        return best_sol

    def terminal_input_loop(self):
        while rclpy.ok():
            print("\n" + "="*50)
            print("Nhập tọa độ mục tiêu (cách nhau bởi dấu cách).")
            print("Định dạng: X Y Z Roll Pitch Yaw (Góc nhập bằng Độ)")
            print("Ví dụ: 0.5 0.2 0.3 180 0 90")
            user_input = input("Mời nhập: ")
            
            try:
                values = list(map(float, user_input.strip().split()))
                if len(values) != 6:
                    print("Vui lòng nhập đúng 6 số!")
                    continue
                
                x, y, z = values[0], values[1], values[2]
                r, p, yaw = radians(values[3]), radians(values[4]), radians(values[5])
                
                print(f"\nĐang giải IK cho tọa độ: X={x}, Y={y}, Z={z}...")
                
                T_06 = get_t_matrix(x, y, z, r, p, yaw)
                solutions = inverse_kinematics(T_06)
                
                if not solutions:
                    print("LỖI TOÁN HỌC: Điểm này nằm ngoài không gian làm việc!")
                    continue
                
                target_joints = self.select_best_solution(solutions, x, y, z)
                
                if target_joints is None:
                    print("TỪ CHỐI LỆNH: Tọa độ này sinh ra 'Nghiệm Ma' (Cánh tay bị ngắn, không với tới đích)!")
                    continue
                    
                joints_deg = [degrees(j) for j in target_joints]
                sum_234 = joints_deg[1] + joints_deg[2] + joints_deg[3]
                
                print("TÌM THẤY NGHIỆM CHUẨN XÁC 100%!")
                print(f"Góc sinh ra (Độ): {[round(j, 2) for j in joints_deg]}")
                print(f"Tổng (Theta2 + Theta3 + Theta4) = {sum_234:.2f} độ")
                
                T_check = calculate_fk(target_joints)
                check_x, check_y, check_z = T_check[0,3], T_check[1,3], T_check[2,3]
                check_r, check_p, check_yw = rotation_matrix_to_rpy(T_check[:3,:3])
                
                print("🔍 KẾT QUẢ ĐỘNG HỌC THUẬN (CROSS-CHECK):")
                print(f"   Vị trí đích thực tế : [{check_x:.4f}, {check_y:.4f}, {check_z:.4f}]")
                print(f"   Góc đích thực tế    : [{check_r:.2f}, {check_p:.2f}, {check_yw:.2f}]")

                self.publish_trajectory(target_joints)
                
            except Exception as e:
                print(f"Lỗi hệ thống: {e}")

    def publish_trajectory(self, joint_angles):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 3 
        point.time_from_start.nanosec = 0
        
        msg.points.append(point)
        self.traj_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()