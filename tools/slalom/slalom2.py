import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import signal
from slalom import Slalom

dt = 0.001
m = 0.015
Tc = 0.001

K = 135

list_K_x = [1]
list_K_y = [0.5, 0.5, 0.5, 0.5]


# list_K_y = [1, 1, 1, 2]

# list_K_y = [10, 20, 20]


class Slalom2(Slalom):

    def __init__(self, v, rad, n, ang1, ang2, ang3, end_pos, slip_gain, type, K, list_K_y):
        self.v = v
        self.rad = rad
        self.ang1 = ang1 * math.pi / 180
        self.ang2 = ang2 * math.pi / 180
        self.ang3 = ang3 * math.pi / 180
        self.ang = self.ang3
        self.base_ang = ang3
        self.end_pos = end_pos
        self.type = type
        self.slip_gain = slip_gain
        self.base_alpha = (2 * v * v) / (rad * rad * self.ang3 / 2)
        self.limit_time_count = 1500
        self.K = K
        self.list_K_y = list_K_y
        self.pow_n = n

    def calc(self, start_ang):
        res = {}
        res["x"] = np.array([0])
        res["y"] = np.array([0])
        res["alpha"] = np.array([0])
        res["w"] = np.array([0])
        tmp_w = 0
        tmp_theta = start_ang * math.pi / 180
        tmp_x = 0
        tmp_y = 0
        mode = 0
        for i in range(1, int(self.limit_time_count + 1)):
            tmp_alpha = 0
            if tmp_theta > self.ang3:
                break
            elif tmp_theta > self.ang2:
                mode = 2
            elif tmp_theta > self.ang1:
                mode = 1
            if tmp_w <= 0 and i > 10:
                break
            if mode == 0:
                tmp_alpha = self.base_alpha
            elif mode == 1:
                tmp_alpha = 0
            if mode == 2:
                tmp_alpha = -self.base_alpha
                diff = math.fabs(tmp_theta - self.ang3)
                if diff < 0.01:
                    diff = 0.01
                if diff * 180 / math.pi < 0.75:
                    tmp_alpha = -tmp_w * tmp_w / (2 * diff)

            tmp_w = tmp_w + tmp_alpha * dt
            tmp_theta = tmp_theta + tmp_w * dt

            tmp_x = tmp_x + self.v * \
                    math.cos(self.start_theta + tmp_theta) * dt
            tmp_y = tmp_y + self.v * \
                    math.sin(self.start_theta + tmp_theta) * dt
            res["x"] = np.append(res["x"], tmp_x)
            res["y"] = np.append(res["y"], tmp_y)
            res["alpha"] = np.append(res["alpha"], tmp_alpha)
            res["w"] = np.append(res["w"], tmp_w)
        # print(np.max(res["w"]) ** 2 * self.rad / 9.81 / 1000)
        self.res = res
        return res

    def calc_slip(self, start_ang):
        res = {}
        res["x"] = np.array([])
        res["y"] = np.array([])
        res["alpha"] = np.array([])
        res["w"] = np.array([])
        res["v"] = np.array([])
        res["vx"] = np.array([])
        res["vy"] = np.array([])
        res["beta"] = np.array([])
        res["acc_y"] = np.array([])
        tmp_w = 0
        tmp_theta = start_ang * math.pi / 180
        tmp_x = 0
        tmp_y = 0
        slip_theta = 0
        ax = 0
        ay = 0
        vx = self.v / 1000
        vy = 0
        Fx = 0
        Fy = 0
        beta = 0
        s = 0
        delta_beta = 0
        old_beta = 0
        mode = 0
        for i in range(1, int(self.limit_time_count + 1)):
            tmp_alpha = 0
            if tmp_theta > self.ang3:
                break
            elif tmp_theta > self.ang2:
                mode = 2
            elif tmp_theta > self.ang1:
                mode = 1
            if tmp_w <= 0 and i > 10:
                break

            if mode == 0:
                tmp_alpha = self.base_alpha
            elif mode == 1:
                tmp_alpha = 0
            elif mode == 2:
                tmp_alpha = -self.base_alpha
                diff = math.fabs(tmp_theta - self.ang3)
                if diff < 0.01:
                    diff = 0.01
                # if diff * 180 / math.pi < 0.75:
                tmp_alpha = -tmp_w * tmp_w / (2 * diff)

            old_w = tmp_w

            tmp_w = tmp_w + tmp_alpha * dt
            tmp_theta = tmp_theta + tmp_w * dt + delta_beta
            # Fx = 0

            err = self.v / 1000 - np.sqrt(vx ** 2 + vy ** 2)
            s = s + err
            Fx = 100.0 * err + 0.01 * s

            Fx = 0
            v2 = np.sqrt(vx ** 2 + vy ** 2)
            tmpK = np.interp(v2 * 1000, list_K_x, self.list_K_y)
            Fy = -tmpK * beta
            # print(Fy, tmpK, v2)
            ax = Fx / m + old_w * vy
            ay = Fy / m - old_w * vx

            vy = vy + ay * dt
            vx = vx + ax * dt

            tmp_v = np.sqrt(vx ** 2 + vy ** 2)

            tmp_x = tmp_x + tmp_v * 1000 * \
                    np.cos(self.start_theta + tmp_theta) * dt
            tmp_y = tmp_y + tmp_v * 1000 * \
                    np.sin(self.start_theta + tmp_theta) * dt

            # tmp_x = tmp_x + vx * 1000 * dt
            # tmp_y = tmp_y + vy * 1000 * dt

            res["v"] = np.append(res["v"], tmp_v)
            res["vx"] = np.append(res["vx"], vx)
            res["vy"] = np.append(res["vy"], vy)

            res["x"] = np.append(res["x"], tmp_x)
            res["y"] = np.append(res["y"], tmp_y)
            res["alpha"] = np.append(res["alpha"], tmp_alpha)
            res["w"] = np.append(res["w"], tmp_w)
            res["acc_y"] = np.append(res["acc_y"], (tmp_v * tmp_w))
            old_beta = beta
            beta = np.arctan2(vy, vx)
            beta = (old_beta / dt - tmp_w) / (1.0 / dt + self.K / tmp_v)
            res["beta"] = np.append(res["beta"], beta)

            delta_beta = beta - old_beta
            # beta = tmp_v * tmp_w

        self.res = res

        return res
