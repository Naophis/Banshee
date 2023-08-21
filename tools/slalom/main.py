import numpy as np
import matplotlib.pyplot as plt

from slalom import Slalom
from plot import Plot
from plotorval import PlotOrval

p = Plot()
po = PlotOrval()

v = 300
show = True
# show = False
K = 135
list_K_y = [0.5]
# K = 13540
# show = False

offset = {
    "prev": 12,
    "after": 0,
    "prev_dia": 20,
    "after_dia": 0,
}

# p.exe("normal", v,show ,0, K, list_K_y,offset)
# p.exe("large", v, show, 0, K, list_K_y,offset)
# p.exe("orval", v, show ,0,  K,list_K_y,offset)
# p.exe("dia45", v, show, 0, K, list_K_y,offset)
# p.exe("dia45", v, show, 1, K, list_K_y,offset)
p.exe("dia45_2", v, show, 0, K, list_K_y, offset)
# p.exe("dia135", v, show ,0,  K,list_K_y,offset)
# p.exe("dia135_2", v, show ,0,  K,list_K_y,offset)
# p.exe("dia90", v, show ,0, K, list_K_y,offset)
