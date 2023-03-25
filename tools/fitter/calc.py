# coding: utf-8
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq

# https://org-technology.com/posts/scipy-least-square-fitting.html


def objectiveFunction(beta):
    r = y - theoreticalValue(beta)
    return r


def theoreticalValue(beta):
    f = beta[0]*x / (beta[1]+x)
    return f


y = np.array([1140, 650, 390, 250])
x = np.array([27, 33, 39, 45])

# x = np.array([0.038, 0.194, 0.425, 0.626, 1.253, 2.500, 3.740])
# y = np.array([0.050, 0.127, 0.094, 0.2122, 0.2729, 0.2665, 0.3317])

initialValue = np.array([0.9, 0.2])
betaID = leastsq(objectiveFunction, initialValue)

print(betaID)

plt.figure()
plt.plot(x, y, 'r.')
plt.plot(x, theoreticalValue(betaID[0]), 'b')
plt.show()
