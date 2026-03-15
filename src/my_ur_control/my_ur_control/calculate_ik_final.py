import math

class ChargingRobotIK:
    def __init__(self):
        # --- 1. THÔNG SỐ CƠ KHÍ "VÀNG" (Đo từ Fusion 360) ---
        self.L_FORWARD = 0.2022  # 202.2 mm (Độ vươn đầu sạc)
        self.L_DOWN    = 0.0812  # 81.2 mm (Độ hạ thấp tâm lỗ)
        
        # --- 2. THÔNG SỐ ROBOT UR10 (DH Parameters) ---
        self.a2 = 0.612    
        self.a3 = 0.5723   
        self.d1 = 0.1273
        self.d4 = 0.163941
        self.d6 = 0.0922

        # --- 3. VỊ TRÍ LẮP ĐẶT TRÊN XE ---
        self.BASE_X = 0.5
        self.BASE_Y = 0.0
        self.BASE_Z = 1.4
        self.BASE_YAW = 180.0 # Robot quay mặt về sau (độ)

    def solve(self, target_x, target_y, target_z, car_x, car_y, car_yaw_deg, station_roll_deg=0.0):
        """
        Tính toán 6 góc khớp để cắm sạc.
        Input:
            - target_x, y, z: Tọa độ TÂM LỖ SẠC (Tuyệt đối trong World)
            - car_x, y, yaw: Tọa độ xe
            - station_roll_deg: Độ nghiêng của trạm sạc (Mặc định 0)
        Output:
            - List[float]: [th1, th2, th3, th4, th5, th6] (Đơn vị: Độ) hoặc None nếu lỗi.
        """
        print(f"\n🚀 TÍNH TOÁN IK (Quy luật -270 độ | Theta5=90)")
        
        # 1. Chuyển đổi tọa độ: World -> Xe -> Robot Base
        car_yaw = math.radians(car_yaw_deg)
        
        # Vector từ xe đến trạm
        dx = target_x - car_x
        dy = target_y - car_y
        
        # Xoay về hệ quy chiếu của xe
        local_x = dx * math.cos(-car_yaw) - dy * math.sin(-car_yaw)
        local_y = dx * math.sin(-car_yaw) + dy * math.cos(-car_yaw)
        
        # Chuyển sang hệ Robot (Robot quay 180 độ so với xe)
        # X_robot ngược chiều X_xe, Y_robot ngược chiều Y_xe
        rob_x = -(local_x - self.BASE_X)
        rob_y = -(local_y - self.BASE_Y)
        rob_z = target_z - self.BASE_Z
        
        # 2. Tính Theta 1 (Base Pan) - Quay vai về hướng trạm
        theta1 = math.atan2(rob_y, rob_x)
        
        # 3. Tính toán hình học phẳng (Planar Geometry)
        # R_tip: Khoảng cách ngang từ tâm vai đến mũi kim
        R_tip = math.sqrt(rob_x**2 + rob_y**2)
        Z_tip = rob_z
        
        # 4. Tính ngược về tâm Wrist 3
        # Mẹo: Tâm Wrist 3 nằm cao hơn Flange đoạn d6 (vì Flange úp đất)
        # Và lùi lại đoạn L_FORWARD
        R_w3 = R_tip - self.L_FORWARD
        Z_w3 = Z_tip + self.L_DOWN + self.d6 

        # 5. Giải Tam Giác (Tìm Theta 2, 3)
        # Bù trừ offset d4 (khoảng cách ngang vai-cổ tay)
        if abs(R_w3) < self.d4:
            print(f"LỖI: Điểm đến quá gần ({R_w3:.3f}m), bị kẹt khớp d4!")
            return None
            
        # Bán kính thực tế cánh tay phải vươn (cạnh đáy tam giác)
        R_planar = math.sqrt(R_w3**2 - self.d4**2)
        # Chiều cao thực tế (cạnh đứng tam giác - trừ chiều cao vai d1)
        H_ver = Z_w3 - self.d1
        
        # Cạnh huyền nối Vai -> Wrist 3
        H = math.sqrt(R_planar**2 + H_ver**2)
        
        # Kiểm tra tầm với vật lý
        if H > (self.a2 + self.a3):
             print(f"LỖI: Xa quá không với tới! (Cần {H:.3f}m, Max {self.a2+self.a3:.3f}m)")
             return None

        # Định lý hàm số Cosin: Tìm góc Khuỷu (Theta 3)
        cos_elbow = (self.a2**2 + self.a3**2 - H**2) / (2 * self.a2 * self.a3)
        cos_elbow = max(-1.0, min(1.0, cos_elbow)) # Kẹp giá trị an toàn
        angle_elbow = math.acos(cos_elbow)
        
        # Chọn nghiệm "Elbow Up" (Gập lên) -> Dấu âm
        theta3 = -(math.pi - angle_elbow) 
        
        # Định lý hàm số Cosin/Sin: Tìm góc Vai (Theta 2)
        # Góc nâng của cạnh huyền H
        angle_H = math.atan2(H_ver, R_planar)
        # Góc lệch alpha bên trong tam giác
        cos_alpha = (self.a2**2 + H**2 - self.a3**2) / (2 * self.a2 * H)
        cos_alpha = max(-1.0, min(1.0, cos_alpha))
        alpha = math.acos(cos_alpha)
        
        # Góc hình học so với phương ngang
        theta2_geom = -(angle_H + alpha) 
        
        # CHUYỂN ĐỔI SANG HỆ UR: -180 là nằm ngang hướng tới trước
        theta2_ur = theta2_geom - math.pi
        # Chuẩn hóa về [-2pi, 2pi]
        if theta2_ur < -2*math.pi: theta2_ur += 2*math.pi

        # 6. Tính Theta 4 (QUY LUẬT CỦA FEN)
        # Tổng 3 góc = -270 độ (-4.712 rad) để mặt bích song song đất
        TARGET_SUM = math.radians(-270)
        theta4_ur = TARGET_SUM - theta2_ur - theta3
        
        # Chuẩn hóa Theta 4
        while theta4_ur < -math.pi: theta4_ur += 2*math.pi
        while theta4_ur > math.pi: theta4_ur -= 2*math.pi

        # 7. Theta 5, 6 (QUY LUẬT KHÓA & XOAY)
        theta5 = math.radians(90) # Khóa ngang (Theo luật fen)
        theta6 = math.radians(station_roll_deg) # Xoay theo độ nghiêng trạm

        # Kết quả cuối cùng
        joints = [
            math.degrees(theta1),
            math.degrees(theta2_ur),
            math.degrees(theta3),
            math.degrees(theta4_ur),
            90.0,
            station_roll_deg
        ]
        
        print(f"✅ KẾT QUẢ: {joints}")
        return joints

# --- PHẦN CHẠY TEST ĐỘC LẬP (Nếu chạy trực tiếp file này) ---
if __name__ == "__main__":
    ik_solver = ChargingRobotIK()
    
    # Test thử một điểm
    ik_solver.solve(
        car_x=0.5, car_y=1.5, car_yaw_deg=0.0,
        target_x=1.5, target_y=1.5, target_z=0.735, # Z đã bù offset
        station_roll_deg=0.0 # Thử đổi thành 15 độ nếu muốn test xoay
    )