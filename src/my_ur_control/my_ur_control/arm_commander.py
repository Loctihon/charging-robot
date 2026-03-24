import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import *
import random
import threading

# ================= FK =================
d1 = 0.1273
a2 = -0.612
a3 = -0.5723
d4 = 0.1639
d5 = 0.1157
d6 = 0.0922

a = [0, a2, a3, 0, 0, 0]
d = [d1, 0, 0, d4, d5, d6]
alph = [pi/2, 0, 0, pi/2, -pi/2, 0]

def fk_all(q):
    T = np.eye(4)
    Ts = []
    for i in range(6):
        ct = cos(q[i]); st = sin(q[i])
        ca = cos(alph[i]); sa = sin(alph[i])
        A = np.array([
            [ct, -st*ca, st*sa, a[i]*ct],
            [st, ct*ca, -ct*sa, a[i]*st],
            [0, sa, ca, d[i]],
            [0,0,0,1]
        ])
        T = T @ A
        Ts.append(T.copy())
    return Ts

# ================= NODE =================
class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_names = [
            'ur10_shoulder_pan_joint',
            'ur10_shoulder_lift_joint',
            'ur10_elbow_joint',
            'ur10_wrist_1_joint',
            'ur10_wrist_2_joint',
            'ur10_wrist_3_joint'
        ]

        # ======= CHỈNH Ở ĐÂY =======
        self.car_center = np.array([0.0, 0.0, 0.5])
        self.car_size   = np.array([1.8, 0.85, 1.0])
        self.base_offset = np.array([0.5, 0, 1.0])

        self.joint_limits = [
            (-2*pi, 2*pi),
            (-2.6, 0),
            (-2.5, 2.5),
            (-pi, pi),
            (-pi, pi),
            (-pi, pi)
        ]

        threading.Thread(target=self.loop, daemon=True).start()

    # ================= COLLISION =================
    def in_box(self, p):
        return all(abs(p - self.car_center) <= self.car_size/2)

    def is_collision(self, q):
        Ts = fk_all(q)
        for T in Ts:
            p = T[:3,3] + self.base_offset
            if self.in_box(p):
                return True
        return False

    def is_path_valid(self, q1, q2):
        for t in np.linspace(0,1,10):
            q = (1-t)*np.array(q1) + t*np.array(q2)
            if self.is_collision(q):
                return False
        return True

    # ================= RRT =================
    def sample(self):
        return [random.uniform(l[0], l[1]) for l in self.joint_limits]

    def dist(self, q1, q2):
        return np.linalg.norm(np.array(q1)-np.array(q2))

    def nearest(self, tree, q):
        return min(tree, key=lambda n: self.dist(n, q))

    def steer(self, q1, q2, step=0.2):
        q1 = np.array(q1); q2 = np.array(q2)
        direction = q2 - q1
        if np.linalg.norm(direction) < step:
            return q2.tolist()
        return (q1 + direction/np.linalg.norm(direction)*step).tolist()

    def rrt(self, start, goal):
        tree = [start]
        parent = {tuple(start): None}

        for _ in range(2000):
            q_rand = self.sample()
            q_near = self.nearest(tree, q_rand)
            q_new = self.steer(q_near, q_rand)

            if not self.is_collision(q_new) and self.is_path_valid(q_near, q_new):
                tree.append(q_new)
                parent[tuple(q_new)] = tuple(q_near)

                if self.dist(q_new, goal) < 0.3:
                    parent[tuple(goal)] = tuple(q_new)
                    return self.extract_path(parent, start, goal)
        return None

    def extract_path(self, parent, start, goal):
        path = [goal]
        cur = tuple(goal)
        while cur != tuple(start):
            cur = parent[cur]
            path.append(list(cur))
        return path[::-1]

    # ================= PUBLISH =================
    def publish(self, path):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        for i, q in enumerate(path):
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start.sec = i
            msg.points.append(pt)

        self.pub.publish(msg)

    # ================= LOOP =================
    def loop(self):
        while rclpy.ok():
            print("\nNhập start (6 góc rad):")
            start = list(map(float, input().split()))

            print("Nhập goal (6 góc rad):")
            goal = list(map(float, input().split()))

            if self.is_collision(start):
                print("Start bị collision")
                continue
            if self.is_collision(goal):
                print("Goal bị collision")
                continue

            path = self.rrt(start, goal)

            if path:
                print("Tìm được path!")
                self.publish(path)
            else:
                print("Không tìm được path")

# ================= MAIN =================
def main():
    rclpy.init()
    node = RRTPlanner()
    rclpy.spin(node)

if __name__ == '__main__':
    main()