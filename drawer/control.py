import math
import numpy as np

# 轴距
L = 0.4
# 前视距离
k = 0.1
# 前视距离基数
Lfc = 0.1
# 周期
dt = 0.1

class State:
    def __init__(self, x=0., y=0., yaw=0., v=0.):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, delta):
        l = self.v * dt
        sy = math.sin(self.yaw)
        cy = math.cos(self.yaw)

        if delta == 0.:
            self.x += l * cy
            self.y += l * sy
            return

        R = L / math.tan(delta)
        alpha = l / R

        sa = math.sin(alpha)
        ca = math.cos(alpha)

        self.x += R * sa * sy + R * (1 - ca) * cy
        self.y += R * sa * cy - R * (1 - ca) * sy

        self.yaw += alpha



class Control:
    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.state = State(v=0.1)


    def get_target(self, curr_pos):

        def distance(a, b):
            dx = a[0] - b[0]
            dy = a[1] - b[1]
            return math.sqrt(dx ** 2 + dy ** 2)

        dd = [ distance(curr_pos, t) for t in self.trajectory]
        idx = dd.index(min(dd))

        l = 0.
        Lf = k * self.state.v + Lfc

        while Lf > l and (idx+1 < len(dd)):
            l += distance(self.trajectory[idx+1], self.trajectory[idx])
            idx += 1

        return min(idx, len(self.trajectory)-1)


    # delta't = arctan(2*L*sin(alpha't) / k*v't+Lfc)
    def pure_pursuit(self, last_idx):
        idx = max(last_idx, self.get_target([self.state.x, self.state.y]))
        t = self.trajectory[idx]

        alpha = math.atan2(t[1] - self.state.y, t[0] - self.state.x) - self.state.yaw
        Lf = k * self.state.v + Lfc
        delta = math.atan2(2 * L * math.sin(alpha) / Lf, 1.0)

        return idx, delta


    def run(self):
        print(self.trajectory)
        idx = self.get_target([self.state.x, self.state.y])
        i=0
        while i < 100 and idx < len(self.trajectory)-1:
            idx, delta = self.pure_pursuit(idx)
            self.state.update(delta)
            print('idx: ', idx)
            print('x: ', self.state.x, ' y: ', self.state.y)
            i+=1

