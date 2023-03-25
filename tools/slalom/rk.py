import numpy as np
import matplotlib.pyplot as plt
import math


def dy(x, y):
    return x**2


def derivative(f, x, h):
    return (f(x+h)-f(x))/h


h = 0.001


def rk(x, y, h):
    k1 = dy(x, y)
    k2 = dy(x+h/2, y+h*k1/2)
    k3 = dy(x+h/2, y+h*k2/2)
    k4 = dy(x+h, y+h*k3)
    ky = y + h*(k1+2*k2+2*k3+k4)/6
    return ky


N = 2
s = 0.11281250000001165
dt = h

alpha = 1000.0/52


def dy2(x, t):
    z = 1
    t = t / s
    if t == 0:
        return 0
    P = math.pow((t - z), N - z)
    Q = P * (t - z)
    res = -N * P / ((Q - z) * (Q - z)) * \
        math.pow(math.exp(1), z + z / (Q - z)) / s
    return alpha * res*dt


list = np.arange(h, s, h)

y = [dy2(0, dt)]


def rk2(x, t):
    k1 = dy2(x, t)
    k2 = dy2(x, t+k1/2*dt)
    k3 = dy2(x, t+k2/2*dt)
    k4 = dy2(x, t+k3*dt)
    res = x + t*(k1+2*k2+2*k3+k4)/6
    return res


for t in np.arange(h, s, h):
    y.append(rk2(y[-1], t))

x = np.arange(0, s, h)

plt.plot(x, y)
plt.show()
