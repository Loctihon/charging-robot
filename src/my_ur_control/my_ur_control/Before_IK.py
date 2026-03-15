import math

# --- 1. SỐ ĐO CHUẨN TỪ FUSION 360 ---
L_FORWARD = 0.2022  # 202.2 mm
L_DOWN    = 0.0812  # 81.2 mm (Đã update số chuẩn)

# --- 2. THÔNG SỐ ROBOT ---
a2 = 0.612    
a3 = 0.5723   
d1 = 0.1273
d4 = 0.163941
d6 = 0.0922

# Vị trí lắp Robot trên xe
BASE_X = 0.5
BASE_Y = 0.0
BASE_Z = 1.4

def solve_ik_final(target_x, target_y, target_z, car_x, car_y, car_yaw_deg):
    print(f"\n🚀 TÍNH TOÁN IK (FINAL FIX)")
    print(f"   Trạm sạc: Z={target_z}m | Đầu sạc hạ: {L_DOWN*1000}mm")
    
    # 1. Tọa độ mũi sạc so với Vai Robot
    car_yaw = math.radians(car_yaw_deg)
    dx = target_x - car_x
    dy = target_y - car_y
    local_x = dx * math.cos(-car_yaw) - dy * math.sin(-car_yaw)
    local_y = dx * math.sin(-car_yaw) + dy * math.cos(-car_yaw)
    
    rob_x = -(local_x - BASE_X)
    rob_y = -(local_y - BASE_Y)
    rob_z = target_z - BASE_Z
    
    # 2. Theta 1 (Quay vai)
    theta1 = math.atan2(rob_y, rob_x)
    
    # 3. Tính toán trong mặt phẳng cánh tay
    R_tip = math.sqrt(rob_x**2 + rob_y**2)
    Z_tip = rob_z
    
    # 4. Tính tâm Wrist 3
    # Mẹo: Tâm Wrist 3 cao hơn Flange đoạn d6 (vì Flange úp đất)
    R_w3 = R_tip - L_FORWARD
    Z_w3 = Z_tip + L_DOWN + d6 

    # 5. Giải Tam Giác (Tìm Theta 2, 3)
    # Bù trừ offset d4 (khoảng cách ngang vai-cổ tay)
    # Vì cấu trúc UR, bán kính với thực tế trong mặt phẳng khớp là cạnh huyền của (R, d4)
    # R_planar^2 + d4^2 = R_w3^2  => R_planar = sqrt(R_w3^2 - d4^2)
    if abs(R_w3) < d4:
        print("❌ LỖI: Điểm đến quá gần, bị kẹt d4!")
        return
        
    R_planar = math.sqrt(R_w3**2 - d4**2)
    H_ver = Z_w3 - d1
    
    # Cạnh huyền tam giác a2-a3
    H = math.sqrt(R_planar**2 + H_ver**2)
    
    # Góc Khuỷu (Theta 3)
    cos_elbow = (a2**2 + a3**2 - H**2) / (2 * a2 * a3)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    angle_elbow = math.acos(cos_elbow)
    theta3 = -(math.pi - angle_elbow) # Gập lên (Elbow Up)
    
    # Góc Vai (Theta 2) - Toán học
    angle_H = math.atan2(H_ver, R_planar)
    cos_alpha = (a2**2 + H**2 - a3**2) / (2 * a2 * H)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)
    
    theta2_geom = -(angle_H + alpha) # Góc so với phương ngang (Geometric)
    
    # --- QUAN TRỌNG: CHUYỂN ĐỔI SANG HỆ UR ---
    # Với UR, -180 là nằm ngang hướng tới trước.
    # Nên góc UR = Góc Toán - 180
    theta2_ur = theta2_geom - math.pi
    
    # Chuẩn hóa về [-2pi, 2pi]
    if theta2_ur < -2*math.pi: theta2_ur += 2*math.pi

    # 6. Tính Theta 4 (Dựa trên tổng -270)
    # Theta2 + Theta3 + Theta4 = -270 (-4.71 rad)
    TARGET_SUM = math.radians(-270)
    theta4_ur = TARGET_SUM - theta2_ur - theta3
    
    # Chuẩn hóa Theta 4
    while theta4_ur < -math.pi: theta4_ur += 2*math.pi
    while theta4_ur > math.pi: theta4_ur -= 2*math.pi

    # 7. Theta 5, 6
    theta5 = math.radians(90)
    theta6 = math.radians(0)

    print("\n✅ KẾT QUẢ GÓC KHỚP CHUẨN (Copy vào manual_alignment):")
    print(f"   Theta 1: {math.degrees(theta1):.2f}")
    print(f"   Theta 2: {math.degrees(theta2_ur):.2f} (Đã fix)")
    print(f"   Theta 3: {math.degrees(theta3):.2f}")
    print(f"   Theta 4: {math.degrees(theta4_ur):.2f}")
    print(f"   Theta 5: 90.00")
    print(f"   Theta 6: 0.00")

# --- CHẠY THỬ ---
solve_ik_final(
    car_x=0.5, car_y=1.5, car_yaw_deg=0.0,
    target_x=1.5, target_y=1.5, target_z=0.7
)