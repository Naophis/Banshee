import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import signal

dt = 0.001
m = 0.015
Tc = 0.001

# K = 175
# K = 135
# K = 135
# K = 200

list_K_x = [1]


# list_K_y = [1, 1, 1, 2]

# list_K_y = [10, 20, 20]


class Slalom:
    base_time = 0
    v = 0
    rad = 0
    ang = 0
    Et = 0
    limit_time_count = 0
    base_alpha = 0
    pow_n = 0
    start_theta = 0
    res = {}
    end_pos = {}
    start_offset = 0
    end_offset = 0
    base_ang = 0
    type = ""
    start_offset_list = []
    end_offset_list = []
    turn_offset = {"x": 0, "y": 0}
    cell_size = 90
    half_cell_size = 45
    slip_gain = 50
    K = 1
    list_K_y = []

    def __init__(self, v, rad, n, ang, end_pos, slip_gain, type, K, list_K_y):
        self.v = v
        self.rad = rad
        self.ang = ang * math.pi / 180
        self.base_ang = ang
        self.pow_n = n
        self.end_pos = end_pos
        self.type = type
        self.slip_gain = slip_gain
        self.K = K
        self.list_K_y = list_K_y
        if n == 2:
            self.Et = 0.603450161218938087668
        elif n == 4:
            self.Et = 0.763214618198974433973
        self.base_alpha = v / rad

    def calc_base_time(self):
        c = 0
        t1 = 0
        tmp_dt = 0.001 / 64

        while c < 10000000:
            t1 = t1 + tmp_dt
            if (2 * self.v / self.rad * self.Et * t1) >= self.ang:
                self.base_time = t1
                self.limit_time_count = t1 * 2 / dt
                return
            c = c + 1

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
        for i in range(1, int(self.limit_time_count + 1)):
            tmp_alpha = self.base_alpha * \
                        self.calc_neipire(dt * i, self.base_time, self.pow_n)
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
        for i in range(1, int(self.limit_time_count + 1)):
            tmp_alpha = self.base_alpha * \
                        self.calc_neipire(dt * i, self.base_time, self.pow_n)
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

    def calcnormal(self, start_ang):
        res = {}
        res["x"] = np.array([0])
        res["y"] = np.array([0])
        res["alpha"] = np.array([0])
        res["w"] = np.array([0])
        tmp_w = 0
        tmp_theta = 0
        tmp_x = 0
        tmp_y = 0
        state = 0
        alpha = 2 * self.v * self.v / (self.rad * self.rad * self.ang / 3)
        while True:
            tmp_alpha = 0
            if state == 0:
                tmp_alpha = alpha
            elif state == 1:
                tmp_alpha = 0
            elif state == 2:
                tmp_alpha = -alpha

            tmp_w = tmp_w + tmp_alpha * dt
            tmp_theta = tmp_theta + tmp_w * dt

            tmp_x = tmp_x + self.v * \
                    math.cos(self.start_theta + tmp_theta) * dt
            tmp_y = tmp_y + self.v * \
                    math.sin(self.start_theta + tmp_theta) * dt

            if state == 0:
                if tmp_theta > (self.ang / 3):
                    state = 1
            elif state == 1:
                if tmp_theta > (self.ang * 2 / 3):
                    state = 2
            elif state == 2:
                if tmp_w < 0:
                    state = 3
                if tmp_theta > (self.ang):
                    state = 3

            if state == 3:
                break

            res["x"] = np.append(res["x"], tmp_x)
            res["y"] = np.append(res["y"], tmp_y)
            res["alpha"] = np.append(res["alpha"], tmp_alpha)
            res["w"] = np.append(res["w"], tmp_w)

        self.res = res
        return res

    def calc_slip_normalturn(self, start_ang):
        res = {}
        res["x"] = np.array([0])
        res["y"] = np.array([0])
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
        state = 0
        alpha = 2 * self.v * self.v / (self.rad * self.rad * self.ang / 3)
        while True:
            tmp_alpha = 0
            if state == 0:
                tmp_alpha = alpha
            elif state == 1:
                tmp_alpha = 0
            elif state == 2:
                tmp_alpha = -alpha

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
            beta = (old_beta / dt - tmp_w) / (1.0 / dt + K / tmp_v)
            res["beta"] = np.append(res["beta"], beta)

            delta_beta = beta - old_beta

            if state == 0:
                if tmp_theta > (self.ang / 3):
                    state = 1
            elif state == 1:
                if tmp_theta > (self.ang * 2 / 3):
                    state = 2
            elif state == 2:
                if tmp_w < 0:
                    state = 3
                if tmp_theta > (self.ang):
                    state = 3

            if state == 3:
                break

        self.res = res

        return res

    def calc_neipire(self, t, s, N):
        z = 1
        t = t / s
        P = math.pow((t - z), N - z)
        Q = P * (t - z)
        res = -N * P / ((Q - z) * (Q - z)) * \
              math.pow(math.exp(1), z + z / (Q - z)) / s
        if t == 0:
            return 0
        return res

    def calc_offset_dist(self, start_pos_x, start_pos_y, type, offset):
        a = math.sin(self.ang)
        b = math.cos(self.ang)
        if self.ang == 0:
            a = 1
            b = 0
        end_x = self.res["x"][-1]
        end_y = self.res["y"][-1]

        self.end_offset = (self.end_pos["y"] - end_y - start_pos_y[0]) / a
        self.start_offset = (self.end_pos["x"] - end_x) - self.end_offset * b
        # start_offset=0
        # end_offset=0
        end_offset = (self.end_pos["y"] - end_y) / a
        start_offset = (self.end_pos["x"] - end_x) - end_offset * b
        prev_path_x = [0, 0]
        prev_path_y = [0, 0]
        after_path_x = [0, 0]
        after_path_y = [0, 0]

        if self.type == "normal":
            end_offset = (self.end_pos["y"] - end_y)
            start_offset = (self.end_pos["x"] - end_x)
            prev_path_x = [0, start_offset]
            prev_path_y = [0, 0]
            after_path_x = [end_x, end_x + end_offset * b]
            after_path_y = [end_y, end_y + end_offset * a]
        elif self.type == "large":
            end_offset = (self.end_pos["y"] - end_y - offset["after"])
            start_offset = (self.end_pos["x"] - end_x + offset["prev"])

            prev_path_x = [-offset["prev"], start_offset - offset["prev"]]
            prev_path_y = [0, 0]
            after_path_x = [end_x, end_x + end_offset * b]
            after_path_y = [end_y, end_y + end_offset * a]
        elif self.type == "orval":
            pass
        elif self.type == "dia45":
            end_offset = (self.end_pos["y"] - end_y) / a
            start_offset = (self.end_pos["x"] - end_x + offset["prev"]) - end_offset * b
            prev_path_x = [-offset["prev"], start_offset - offset["prev"]]
            prev_path_y = [0, 0]
            after_path_x = [end_x, end_x + end_offset * b]
            after_path_y = [end_y, end_y + end_offset * a]
        elif self.type == "dia135":
            end_offset = (self.end_pos["y"] - end_y) / a
            start_offset = (self.end_pos["x"] - end_x + offset["prev"]) - end_offset * b
            prev_path_x = [-offset["prev"], start_offset - offset["prev"]]
            prev_path_y = [0, 0]
            after_path_x = [end_x, end_x + end_offset * b]
            after_path_y = [end_y, end_y + end_offset * a]

        elif self.type == "dia45_2":

            start_offset = (self.half_cell_size - end_x) / a + offset["prev_dia"]
            end_offset = (self.cell_size - end_y) - (start_offset - offset["prev_dia"]) * a

            prev_path_x = [-offset["prev_dia"] * a, (start_offset - offset["prev_dia"]) * b]
            prev_path_y = [-offset["prev_dia"] * a, (start_offset - offset["prev_dia"]) * a]
            after_path_x = [end_x, end_x]
            after_path_y = [end_y, end_y + end_offset]

            pass

        elif self.type == "dia135_2":

            start_offset = (self.cell_size - end_y) / b + offset["prev"]
            end_offset = math.fabs(self.half_cell_size + end_x + abs((start_offset - offset["prev"]) * a))

            prev_path_x = [-offset["prev_dia"] * a, abs((start_offset - offset["prev_dia"]) * a)]
            prev_path_y = [-offset["prev_dia"] * a, abs((start_offset - offset["prev_dia"]) * b)]
            after_path_x = [end_x, end_x - end_offset]
            after_path_y = [end_y, end_y]

            pass
        elif self.type == "dia90":
            self.half_cell_size = 0
            end_offset = (self.cell_size / math.sqrt(2) - end_y)
            start_offset = (self.cell_size / math.sqrt(2) - end_x) + offset["prev_dia"]

            prev_path_x = [-offset["prev_dia"], start_offset - offset["prev_dia"]]
            prev_path_y = [0, 0]
            after_path_x = [end_x, end_x + end_offset * b]
            after_path_y = [end_y, end_y + end_offset * a]

            # prev_path_x = [-offset["prev"], start_offset - offset["prev"]]

        after_path_x2 = [after_path_x[0] + prev_path_x[1], after_path_x[1] + prev_path_x[1]]
        after_path_y2 = [after_path_y[0] + prev_path_y[1], after_path_y[1] + prev_path_y[1]]
        res = {}

        res["turn_offset_x"] = prev_path_x[1]  # ターンの原点x
        res["turn_offset_y"] = prev_path_y[1]  # ターンの原点y

        res["prev_path_x"] = prev_path_x  # [x0, x1]
        res["prev_path_y"] = prev_path_y  # [y0, y1]
        res["after_path_x"] = after_path_x
        res["after_path_y"] = after_path_y
        res["after_path_x2"] = after_path_x2
        res["after_path_y2"] = after_path_y2
        res["prev_dist"] = self.calc_dist(prev_path_x, prev_path_y)
        res["after_dist"] = self.calc_dist(after_path_x, after_path_y)
        return res

    def calc_dist(self, list_x, list_y):
        d2 = (list_x[1] - list_x[0]) ** 2 + (list_y[1] - list_y[0]) ** 2
        if d2 > 0:
            return math.sqrt(d2)
        return 0
