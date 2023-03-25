import numpy as np
import matplotlib.pyplot as plt
import math

from slalom import Slalom
from matplotlib import gridspec


plot_row = 5
plot_col = 2

fig = plt.figure(dpi=200)
trj = plt.subplot2grid((plot_row, plot_col), (0, 0), rowspan=5)
trj.plot([0, 1], [2, 4])

trj2 = plt.subplot2grid((plot_row, plot_col), (0, 1), rowspan=1)
trj2.plot([0, 0.4], [2, 4])
plt.show()
