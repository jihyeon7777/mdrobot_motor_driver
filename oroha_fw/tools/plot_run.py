#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import sys

df = pd.read_csv(sys.argv[1] if len(sys.argv) > 1 else "run.csv")
t = (df["t_us"] - df["t_us"].iloc[0]) / 1e6   # 시작 기준 초

fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
ax[0].plot(t, df["V_bus_V"]);  ax[0].set_ylabel("V_bus [V]")
ax[1].plot(t, df["I_L_A"], label="I_L")
ax[1].plot(t, df["I_R_A"], label="I_R")
ax[1].set_ylabel("Current [A]"); ax[1].legend()
ax[2].plot(t, df["P_W"]);      ax[2].set_ylabel("Power [W]")
ax[2].set_xlabel("time [s]")
plt.tight_layout()
plt.show()