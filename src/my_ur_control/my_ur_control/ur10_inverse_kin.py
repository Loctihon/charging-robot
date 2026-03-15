import numpy as np
from math import *

d1 = 0.1273
a2 = -0.612
a3 = -0.5723
d4 = 0.1639
d5 = 0.1157
d6 = 0.0922

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
    c_th = np.zeros((6, 8))
    s_th = np.zeros((6, 8))

    # --- TINH THETA 1 ---
    P_05 = T_06 * np.matrix([0,0,-d6,1]).T
    psi = atan2(P_05[1], P_05[0])
    phi = acos(d4 / sqrt(P_05[0]*P_05[0] + P_05[1]*P_05[1]))
    
    # 2 trường hợp của vai (Trái/Phải)
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
        p6x = T_16[0,3]
        p6y = T_16[1,3]
        
        # 2 trường hợp của cổ tay
        val = (P_16z - d1) / d6
        if abs(val) > 1: val = 1 # Kẹp giá trị trong [-1, 1]
        
        th[4, c:c+2] = -acos(val)
        th[4, c+2:c+4] = acos(val)

    th = th.real

    # --- TINH THETA 6 ---
    # CHưa có trường hợp khâu này phai luôn song song mặt đất
    cl = [0, 2, 4, 6]
    for i in range(len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(ah(1,th,c))
        T_16 = np.linalg.inv( T_10 * T_06 )
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
        
        # Định lý hàm số cos
        costh3 = ((P_13[0]**2 + P_13[1]**2) - a2**2 - a3**2) / (2*a2*a3)
        if abs(costh3) > 1: costh3 = 1 # Kẹp giá trị
        
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
        
        # Theta 2
        th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3*sin(th[2,c])/sqrt(P_13[0]**2 + P_13[1]**2))
        
        # Theta 4
        T_32 = np.linalg.inv(ah(3,th,c))
        T_21 = np.linalg.inv(ah(2,th,c))
        T_34 = T_32 * T_21 * T_14
        th[3, c] = atan2(T_34[1,0], T_34[0,0])

    th = th.real
    
    # Chuyển đổi kết quả về dạng List dễ dùng
    solutions = []
    for i in range(8):
        solutions.append(th[:, i].tolist())
        
    return solutions

# Các biến hỗ trợ tính toán
a = [0, a2, a3, 0, 0, 0]
d = [d1, 0, 0, d4, d5, d6]
alph = [pi/2, 0, 0, pi/2, -pi/2, 0]