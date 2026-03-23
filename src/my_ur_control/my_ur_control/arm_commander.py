import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from math import *
import threading

# ================= DH PARAM =================
d1 = 0.1273
a2 = -0.612
a3 = -0.5723
d4 = 0.1639
d5 = 0.1157
d6 = 0.0922

a = [0, a2, a3, 0, 0, 0]
d = [d1, 0, 0, d4, d5, d6]
alph = [pi/2, 0, 0, pi/2, -pi/2, 0]

# ================= KINEMATICS =================
def ah(n, th, c):
    T_a = np.identity(4)
    T_a[0,3] = a[n-1]

    T_d = np.identity(4)
    T_d[2,3] = d[n-1]

    Rzt = np.array([
        [cos(th[n-1,c]), -sin(th[n-1,c]), 0, 0],
        [sin(th[n-1,c]),  cos(th[n-1,c]), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    Rxa = np.array([
        [1, 0, 0, 0],
        [0, cos(alph[n-1]), -sin(alph[n-1]), 0],
        [0, sin(alph[n-1]),  cos(alph[n-1]), 0],
        [0, 0, 0, 1]
    ])

    return T_d @ Rzt @ T_a @ Rxa


def inverse_kinematics(T_06):
    th = np.zeros((6, 8))

    # theta1
    P_05 = T_06 @ np.array([0,0,-d6,1])
    psi = atan2(P_05[1], P_05[0])
    r = sqrt(P_05[0]**2 + P_05[1]**2)

    if r < d4: return []
    phi = acos(d4 / r)

    th[0,0:4] = pi/2 + psi + phi
    th[0,4:8] = pi/2 + psi - phi

    # theta5
    for c in [0,4]:
        T_10 = np.linalg.inv(ah(1,th,c))
        T_16 = T_10 @ T_06
        val = (T_16[2,3] - d4) / d6
        val = np.clip(val, -1, 1)

        th[4,c:c+2] = -acos(val)
        th[4,c+2:c+4] = acos(val)

    # theta6
    for c in [0,2,4,6]:
        if abs(sin(th[4,c])) < 1e-6: continue
        T_10 = np.linalg.inv(ah(1,th,c))
        T_16 = np.linalg.inv(T_10 @ T_06)

        th[5,c:c+2] = atan2(
            -T_16[1,2]/sin(th[4,c]),
             T_16[0,2]/sin(th[4,c])
        )

    # theta3
    for c in [0,2,4,6]:
        T_10 = np.linalg.inv(ah(1,th,c))
        T_65 = ah(6,th,c)
        T_54 = ah(5,th,c)
        T_14 = (T_10 @ T_06) @ np.linalg.inv(T_54 @ T_65)

        P_13 = T_14 @ np.array([0,-d4,0,1])
        P_13 = P_13[:3]

        D = (P_13[0]**2 + P_13[1]**2 - a2**2 - a3**2) / (2*a2*a3)
        D = np.clip(D, -1, 1)

        th[2,c] = acos(D)
        th[2,c+1] = -acos(D)

    # theta2,4
    for c in range(8):
        T_10 = np.linalg.inv(ah(1,th,c))
        T_65 = np.linalg.inv(ah(6,th,c))
        T_54 = np.linalg.inv(ah(5,th,c))

        T_14 = (T_10 @ T_06) @ T_65 @ T_54
        P_13 = T_14 @ np.array([0,-d4,0,1])
        P_13 = P_13[:3]

        r = sqrt(P_13[0]**2 + P_13[1]**2)
        if r < 1e-6: continue

        val = a3*sin(th[2,c]) / r
        val = np.clip(val, -1, 1)

        th[1,c] = -atan2(P_13[1], -P_13[0]) + asin(val)

        T_32 = np.linalg.inv(ah(3,th,c))
        T_21 = np.linalg.inv(ah(2,th,c))
        T_34 = T_32 @ T_21 @ T_14

        th[3,c] = atan2(T_34[1,0], T_34[0,0])

    return [th[:,i].tolist() for i in range(8)]


def get_T(x,y,z,r,p,yaw):
    Rx = np.array([
        [1,0,0,0],
        [0,cos(r),-sin(r),0],
        [0,sin(r),cos(r),0],
        [0,0,0,1]
    ])
    Ry = np.array([
        [cos(p),0,sin(p),0],
        [0,1,0,0],
        [-sin(p),0,cos(p),0],
        [0,0,0,1]
    ])
    Rz = np.array([
        [cos(yaw),-sin(yaw),0,0],
        [sin(yaw), cos(yaw),0,0],
        [0,0,1,0],
        [0,0,0,1]
    ])

    T = np.eye(4)
    T[0,3],T[1,3],T[2,3] = x,y,z

    return T @ Rz @ Ry @ Rx


def calculate_fk(q):
    T = np.eye(4)
    for i in range(6):
        ct, st = cos(q[i]), sin(q[i])
        ca, sa = cos(alph[i]), sin(alph[i])

        A = np.array([
            [ct, -st*ca, st*sa, a[i]*ct],
            [st,  ct*ca,-ct*sa, a[i]*st],
            [0,   sa,    ca,    d[i]],
            [0,   0,     0,     1]
        ])
        T = T @ A
    return T

# ================= NODE =================
class ArmNode(Node):
    def __init__(self):
        super().__init__('ur10_safe_ik')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.cb,
            10)

        self.joint_names = [
            'ur10_shoulder_pan_joint',
            'ur10_shoulder_lift_joint',
            'ur10_elbow_joint',
            'ur10_wrist_1_joint',
            'ur10_wrist_2_joint',
            'ur10_wrist_3_joint'
        ]

        self.current = None
        self.max_delta = radians(20)

        threading.Thread(target=self.loop, daemon=True).start()

    def cb(self, msg):
        m = dict(zip(msg.name, msg.position))
        try:
            self.current = np.array([m[j] for j in self.joint_names])
        except:
            pass

    def norm(self, a):
        return (a + pi) % (2*pi) - pi

    def select(self, sols, x,y,z):
        if self.current is None:
            print("❌ No joint_state")
            return None

        valid = []

        for sol in sols:
            sol = np.array([self.norm(s) for s in sol])

            delta = np.array([self.norm(sol[i]-self.current[i]) for i in range(6)])
            if np.any(np.abs(delta) > self.max_delta):
                continue

            T = calculate_fk(sol)
            err = np.linalg.norm(T[:3,3] - [x,y,z])

            if err < 0.005:
                valid.append(sol)

        if not valid:
            return None

        best = min(valid, key=lambda s: np.linalg.norm(s-self.current))
        return best

    def loop(self):
        while rclpy.ok():
            try:
                s = input(">>> X Y Z R P Y(deg): ")
                v = list(map(float, s.split()))
                if len(v)!=6: continue

                x,y,z = v[:3]
                r,p,yw = map(radians, v[3:])

                T = get_T(x,y,z,r,p,yw)
                sols = inverse_kinematics(T)

                if not sols:
                    print("❌ No IK")
                    continue

                q = self.select(sols,x,y,z)

                if q is None:
                    print("❌ Unsafe")
                    continue

                print("✅", [round(degrees(i),1) for i in q])
                self.send(q)

            except Exception as e:
                print("ERR:", e)

    def send(self, q):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = q.tolist()
        pt.time_from_start.sec = 2

        msg.points.append(pt)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ArmNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()