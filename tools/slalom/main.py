import numpy as np
import matplotlib.pyplot as plt

from slalom import Slalom
from plot import Plot
from plotorval import PlotOrval

import sys
import os
import yaml


def read_yaml(filename):
    # 現在のスクリプトのディレクトリを取得
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # ファイルパスを結合
    filepath = os.path.join(current_dir, filename)

    with open(filepath, 'r') as file:
        data = yaml.safe_load(file)
    return data

data = read_yaml("../param_tuner/profile/hardware.yaml")

p = Plot()
po = PlotOrval()

v = 1400
dia45_mode = 0

hf_cl = 0
# show = True
show = False
if len(sys.argv) > 1:
    v = int(sys.argv[1])
    dia45_mode = int(sys.argv[2])
    hf_cl = 0
    show = False

K = data["slip_param_k2"]
list_K_y = [data["slip_param_K"]]

offset = {
    "prev": 7,
    "after": 0,  # not use
    "prev_dia": 7,
    "after_dia": 0,  # not use
}

# p.exe("normal", v, show, 0, K, list_K_y, offset, hf_cl)
# p.exe("large", v, show, 0, K, list_K_y, offset, hf_cl)
# p.exe("large", v, show, 1, K, list_K_y, offset, hf_cl)
# p.exe("dia45", v, show, dia45_mode, K, list_K_y, offset, hf_cl)
# p.exe("dia45_2", v, show, 0, K, list_K_y, offset, hf_cl)
# p.exe("dia135", v, show, 0,  K, list_K_y, offset, hf_cl)
# p.exe("dia135_2", v, show, 0,  K, list_K_y, offset, hf_cl)
p.exe("dia90", v, show, 0, K, list_K_y, offset, hf_cl)

# p.exe("orval", v, show, 0, K, list_K_y, offset, hf_cl)
# p.exe("dia45", v, show, 0, K, list_K_y, offset, hf_cl)