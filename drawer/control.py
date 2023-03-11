import math
import time

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

# 轴距
L = 0.15
# 前视距离系数
k = 0.1
# 前视距离基数
Lfc = 0.45
# 速度
v = 0.2
# 周期（仅用于模拟）
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

    def set_state(self, pose):
        def q_to_r(q):
            m = [0] * 16
            c = q.x
            d = q.y
            e = q.z
            f = q.w
            g = c + c
            h = d + d
            k = e + e;
            q = c * g;
            l = c * h
            c = c * k
            n = d * h
            d = d * k
            e = e * k
            g = f * g
            h = f * h
            f = f * k;
            m[0] = 1 - (n + e);
            m[4] = l - f;
            m[8] = c + h;
            m[1] = l + f;
            m[5] = 1 - (q + e);
            m[9] = d - g;
            m[2] = c - h;
            m[6] = d + g;
            m[10] = 1 - (q + n);
            m[3] = 0;
            m[7] = 0;
            m[11] = 0;
            m[12] = 0;
            m[13] = 0;
            m[14] = 0;
            m[15] = 1;
            return m

        def m_to_e(m):
            d = lambda x, l, u: max(min(x, u), l)
            e = m
            m = e[0];
            f = e[4]
            g = e[8]
            h = e[1]
            k = e[5]
            l = e[9]
            n = e[2]
            p = e[6]
            e = e[10];
            _x = math.asin(d(p, -1, 1)),
            if .99999 > abs(p):
                _y = math.atan2(-n, e)
                _z = math.atan2(-f, k)
            else:
                _y = 0
                _z = math.atan2(h, m)
            return _z, _x, _y

        self.x = pose.pose.pose.position.x
        self.y = pose.pose.pose.position.y

        y, _, _ = m_to_e(q_to_r(pose.pose.pose.orientation))
        self.yaw = y
        # print('x: ', self.x)
        # print('y: ', self.y)
        # print('yaw: ', self.yaw)



class Control:
    def __init__(self, trajectory):
        self.trajectory = self.interp(trajectory)
        self.state = State(v=v)


    def interp(self, trajectory):

        x = [t[0] for t in trajectory]
        y = [t[1] for t in trajectory]

        ret = []
        for i in range(len(x) - 1):
            dx = x[i+1] - x[i]
            dy = y[i+1] - y[i]
            d = math.sqrt(dx * dx + dy * dy)
            n = int(d / 0.1) + 1

            xs = []
            for j in range(n):
                xs.append(x[i] + j * dx / n)
            ys = []
            for j in range(n):
                ys.append(y[i] + j * dy / n)

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

        delta = min(0.349, delta)
        return idx, delta


    def run(self):
        rospy.init_node('drawer', anonymous=True)
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5) #创建速度话题发布者，'~cmd_vel'='节点名/cmd_vel'
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.state.set_state)

        idx = self.get_target([self.state.x, self.state.y])
        path=[]
        while not rospy.is_shutdown() and idx < len(self.trajectory)-1:
            idx, delta = self.pure_pursuit(idx)
            # print('delta: ', delta)

            twist = Twist()
            twist.linear.x = v
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = delta
            vel_pub.publish(twist)

            # self.state.update(delta)
            # print('curr: ', self.state.x, self.state.y)

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

