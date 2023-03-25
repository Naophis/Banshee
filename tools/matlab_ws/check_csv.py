import pandas as pd

import matplotlib.pyplot as plt

df = pd.read_csv("sysid/logs/latest.csv")

print(df)

print(df["v_c"].mean())
# plt.plot(df["v_c"])
# plt.plot(df["v_l"])
# plt.plot(df["v_r"])

# plt.show()
