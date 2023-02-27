import numpy as np

class Control:

    def __init__(self):
        pass

    def run(self, trajectory):
        t = self.interp(trajectory)
        print(t)

    def interp(self, trajectory):

        x = [t[0] for t in trajectory]
        y = [t[1] for t in trajectory]

        ret = []
        for i in range(len(x) - 1):
            dx = abs(x[i+1] - x[i])
            dy = abs(y[i+1] - y[i])
            d = np.sqrt(dx * dx + dy * dy)
            n = int(d / 0.1) + 1

            xs = np.linspace(x[i], x[i+1], n).tolist()
            ys = np.linspace(y[i], y[i+1], n).tolist()
            for i in range(n):
                ret.append([xs[i], ys[i]])

        return ret
