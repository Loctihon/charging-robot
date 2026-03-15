import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import sys

# Import Class từ file kia
from calculate_ik_final import ChargingRobotIK

class ManualCharger(Node):
    def __init__(self):
        super().__init__('manual_charger_node')
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.ik_solver = ChargingRobotIK()
        self.get_logger().info("Đã kết nối với Robot! Sẵn sàng nhận lệnh.")

    def move_robot(self, joints_deg):
        if joints_deg is None: return
        
        msg = JointTrajectory()
        msg.joint_names = ['ur10_shoulder_pan_joint', 'ur10_shoulder_lift_joint', 'ur10_elbow_joint',
                           'ur10_wrist_1_joint', 'ur10_wrist_2_joint', 'ur10_wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = [math.radians(j) for j in joints_deg]
        point.time_from_start.sec = 3 # Di chuyển nhanh hơn xíu (3s)
        msg.points.append(point)
        
        self.traj_pub.publish(msg)
        print(f"🚀 Góc khớp thực thi: {[round(x, 2) for x in joints_deg]}")

def main():
    rclpy.init()
    node = ManualCharger()
    
    # Z MẶC ĐỊNH (Nếu fen lười nhập số thứ 4)
    # 0.7 (đáy) + 0.035 (tâm lỗ)
    DEFAULT_Z = 0.735 
    
    # Tọa độ Trạm (X, Y cố định, chỉ chỉnh Z)
    STATION_X = 1.5
    STATION_Y = 1.0
    
    try:
        while rclpy.ok():
            print("\n" + "="*60)
            print(f"TRẠM SẠC TẠI: X={STATION_X}, Y={STATION_Y}")
            print(f"Z mặc định: {DEFAULT_Z}m")
            print("-" * 60)
            print("CÚ PHÁP NHẬP LIỆU:")
            print("   Cách 1 (Dùng Z mặc định):  Car_X  Car_Y  Car_Yaw")
            print("   Cách 2 (Tự chỉnh Z):       Car_X  Car_Y  Car_Yaw  Target_Z")
            print("-" * 60)
            
            user_input = input(">> Nhập lệnh (ví dụ: 0.5 1.5 0 0.74): ")
            if user_input.lower() == 'q': break
            
            try:
                parts = user_input.split()
                
                # Biến lưu giá trị nhập
                c_x, c_y, c_yaw, t_z = 0, 0, 0, DEFAULT_Z
                
                if len(parts) == 3:
                    # Nhập 3 số -> Dùng Z mặc định
                    c_x, c_y, c_yaw = map(float, parts)
                    print(f"⚡ Dùng Z mặc định: {t_z}")
                    
                elif len(parts) == 4:
                    # Nhập 4 số -> Dùng Z tùy chỉnh
                    c_x, c_y, c_yaw, t_z = map(float, parts)
                    print(f"🔧 Z TÙY CHỈNH: {t_z}")
                    
                else:
                    print("⚠️ Sai cú pháp! Hãy nhập 3 hoặc 4 số.")
                    continue

                # GỌI IK GIẢI TOÁN
                joints = node.ik_solver.solve(
                    target_x=STATION_X, 
                    target_y=STATION_Y, 
                    target_z=t_z,         # <--- Z được truyền vào đây
                    car_x=c_x, 
                    car_y=c_y, 
                    car_yaw_deg=c_yaw
                )
                
                # GỬI LỆNH
                if joints:
                    node.move_robot(joints)
                    print(f"👀 Check RViz: TCP đang ở độ cao Z = {t_z}")

            except ValueError:
                print("⚠️ Lỗi: Chỉ được nhập số!")
                
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()