from multiprocessing import dummy
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import curve_fit

fig = plt.figure()
L90 = fig.add_subplot(1, 5, 1)
L45 = fig.add_subplot(1, 5, 2)
F = fig.add_subplot(1, 5, 3)
R45 = fig.add_subplot(1, 5, 4)
R90 = fig.add_subplot(1, 5, 5)


def nonlinear_fit(x, a, b):
    return a/np.log(x) - b


def main_plot(fig, file, col_idx, str):

    df = pd.read_csv(file)
    input_csv = df.sort_values('dist', ascending=False)
    dist = input_csv[input_csv.keys()[0]]
    sensor_raw = input_csv[input_csv.keys()[col_idx]]  # csvの何行目のデータを使うか

    param, cov = curve_fit(nonlinear_fit, sensor_raw, dist)
    print("{}: [{}, {}] #cov[{},{}]" .format(str, param[0], param[1],cov[0][0], cov[1][1]))
    min_x = sensor_raw.min()
    max_x = sensor_raw.max()
    x = np.linspace(min_x, max_x, 20)
    y = nonlinear_fit(x, param[0], param[1])

    fig.scatter(sensor_raw, dist)
    fig.plot(x, y, label='3')

    return param


res_l45 = main_plot(L45, "./result_l.csv", 2, "L45")
# res_l45 = main_plot(F, "./result_f.csv", 3, "F")
res_r45 = main_plot(R45, "./result_r.csv", 4, "R45")

res_l90 = main_plot(L90, "./result_f.csv", 1, "L90")
res_r90 = main_plot(R90, "./result_f.csv", 5, "R90")

plt.show()
