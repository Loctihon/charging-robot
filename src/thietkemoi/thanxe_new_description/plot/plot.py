import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. Đọc file CSV
df = pd.read_csv('dijkstra.csv') 

# DỌN DẸP KHOẢNG TRẮNG TÊN CỘT
df.columns = df.columns.str.strip()

# TỰ ĐỘNG TÌM CỘT
col_vx = [c for c in df.columns if 'linear/x' in c][0]
col_vy = [c for c in df.columns if 'linear/y' in c][0]
col_wz = [c for c in df.columns if 'angular/z' in c][0]

# LỌC SẠCH DỮ LIỆU LỖI (NẾU CÓ)
df_clean = df.dropna(subset=[col_vx, col_vy, col_wz]).copy()

# 2. Xử lý thời gian
time_raw = df_clean.iloc[:, 0]  
time = (time_raw - time_raw.iloc[0]).to_numpy()

# Dữ liệu gốc
vx_raw = df_clean[col_vx].to_numpy()
vy_raw = df_clean[col_vy].to_numpy()
wz_raw = df_clean[col_wz].to_numpy()

# =========================================================
# BỘ LỌC LÀM MƯỢT (MOVING AVERAGE) - MÔ PHỎNG QUÁN TÍNH
# =========================================================
# Lấy trung bình mỗi 5 điểm (~0.25 giây) để bo tròn góc mà không bị vọt lố
window_size = 5
vx = pd.Series(vx_raw).rolling(window=window_size, min_periods=1).mean().to_numpy()
vy = pd.Series(vy_raw).rolling(window=window_size, min_periods=1).mean().to_numpy()
wz = pd.Series(wz_raw).rolling(window=window_size, min_periods=1).mean().to_numpy()

# =========================================================
# VẼ HÌNH ĐÃ ĐƯỢC LÀM MƯỢT (Bỏ drawstyle='steps-post')
# =========================================================
plt.style.use('seaborn-whitegrid')
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 12
plt.rcParams['axes.linewidth'] = 1.5

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 4.5), sharex=True)

# ĐỒ THỊ 1: VẬN TỐC TỊNH TIẾN
ax1.plot(time, vx, color='#d62728', linewidth=2, label=r'$v_x$ (Forward)')
ax1.plot(time, vy, color='#1f77b4', linewidth=2, label=r'$v_y$ (Strafe)')

ax1.set_ylabel('Linear Vel (m/s)', fontweight='bold')
ax1.legend(loc='upper right', frameon=True, edgecolor='black')
ax1.grid(True, linestyle='--', alpha=0.7)
ax1.set_title(r'Dijkstra', fontweight='bold')

# ĐỒ THỊ 2: VẬN TỐC GÓC
ax2.plot(time, wz, color='#2ca02c', linewidth=2, label=r'$\omega_z$ (Yaw Rate)')
ax2.set_xlabel('Time (s)', fontweight='bold')
ax2.set_ylabel('Angular Vel (rad/s)', fontweight='bold')
ax2.legend(loc='upper right', frameon=True, edgecolor='black')
ax2.grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.savefig('dijkstra2.png', dpi=1000, bbox_inches='tight')

# In Độ lệch chuẩn từ dữ liệu thô (để đánh giá thuật toán chính xác nhất)
print(f"--- THÔNG SỐ ĐIỀN BẢNG CHƯƠNG 4 ---")
print(f"Standard Deviation of wz (Raw): {np.std(wz_raw):.4f} rad/s")
print("Đã xuất file hình ảnh bo tròn chuẩn đẹp dijkstra.png!")