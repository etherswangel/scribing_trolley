import math
import time

import rospy
from geometry_msgs.msg import Twist

# 轴距
L = 0.4
# 前视距离
k = 0.1
# 前视距离基数
Lfc = 0.1
# 速度
v = 0.1
# 周期
dt = 0.1

class State:
    def __init__(self, x=0., y=0., yaw=0., v=0.):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, delta):
        time.sleep(dt)

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

        self.x += R * sa * cy - R * (1 - ca) * sy
        self.y += R * sa * sy + R * (1 - ca) * cy

        self.yaw += alpha



class Control:
    def __init__(self, trajectory):
        self.trajectory = self.interp(trajectory)
        self.state = State(v=v)


    def interp(self, trajectory):

        x = [t[0] for t in trajectory]
        y = [t[1] for t in trajectory]

        ret = []
        for i in range(len(x) - 1):
            dx = abs(x[i+1] - x[i])
            dy = abs(y[i+1] - y[i])
            d = math.sqrt(dx * dx + dy * dy)
            n = int(d / 0.1) + 1

            xs = []
            for j in range(n):
                xs.append(x[i] + j * dx)
            ys = []
            for j in range(n):
                ys.append(y[i] + j * dy)

            for i in range(n):
                ret.append([xs[i], ys[i]])

        return ret


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
        rospy.init_node('drawer', anonymous=True)
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5) #创建速度话题发布者，'~cmd_vel'='节点名/cmd_vel'

        idx = self.get_target([self.state.x, self.state.y])
        path=[]
        while idx < len(self.trajectory)-1:
            idx, delta = self.pure_pursuit(idx)
            print('delta: ', delta)
            twist = Twist()
            twist.linear.x = v
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = delta
            vel_pub.publish(twist)

            self.state.update(delta)
            print('curr: ', self.state.x, self.state.y)
            path.append([self.state.x, self.state.y])

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        vel_pub.publish(twist)

        return path

