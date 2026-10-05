import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt

df = pd.read_csv("problem4_run.csv")
os.makedirs("pid_plots", exist_ok=True)

t = df["t_s"].to_numpy()
desired_n, desired_e, desired_d = 10.0, 15.0, -20.0
kp = 2.0  
gain_note = f"Kp = {kp} (N, E, D)"

fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
axes[0].plot(t, df["pN"], label="pN")
axes[0].axhline(desired_n, color="black", linestyle="--", linewidth=1, label="desired")
axes[0].set_ylabel("North (m)")
axes[0].legend()
axes[0].grid(True)

axes[1].plot(t, df["pE"], label="pE", color="tab:orange")
axes[1].axhline(desired_e, color="black", linestyle="--", linewidth=1, label="desired")
axes[1].set_ylabel("East (m)")
axes[1].legend()
axes[1].grid(True)

axes[2].plot(t, df["pD"], label="pD", color="tab:green")
axes[2].axhline(desired_d, color="black", linestyle="--", linewidth=1, label="desired")
axes[2].set_ylabel("Down (m)")
axes[2].set_xlabel("Time (s)")
axes[2].legend()
axes[2].grid(True)

fig.suptitle(f"Position vs. Time (Problem 4b) -- {gain_note}")
fig.tight_layout()
fig.savefig("pid_plots/position_vs_time.png", dpi=200)

fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(t, df["eN"], label="eN")
ax.plot(t, df["eE"], label="eE")
ax.plot(t, df["eD"], label="eD")
ax.axhline(0, color="black", linewidth=0.5)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Position error (m)")
ax.set_title(f"Position Error vs. Time (Problem 4b) -- {gain_note}")
ax.grid(True)
ax.legend()
fig.tight_layout()
fig.savefig("pid_plots/error_vs_time.png", dpi=200)

fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
axes[0].plot(t, df["VN"], label="measured VN")
axes[0].plot(t, df["VN_cmd"], label="commanded VN_cmd", linestyle="--")
axes[0].set_ylabel("VN (m/s)")
axes[0].legend()
axes[0].grid(True)

axes[1].plot(t, df["VE"], label="measured VE", color="tab:orange")
axes[1].plot(t, df["VE_cmd"], label="commanded VE_cmd", linestyle="--", color="tab:red")
axes[1].set_ylabel("VE (m/s)")
axes[1].legend()
axes[1].grid(True)

axes[2].plot(t, df["VD"], label="measured VD", color="tab:green")
axes[2].plot(t, df["VD_cmd"], label="commanded VD_cmd", linestyle="--", color="tab:olive")
axes[2].set_ylabel("VD (m/s)")
axes[2].set_xlabel("Time (s)")
axes[2].legend()
axes[2].grid(True)

fig.suptitle(f"Commanded vs. Measured Velocity (Problem 4b) -- {gain_note}")
fig.tight_layout()
fig.savefig("pid_plots/velocity_vs_time.png", dpi=200)

print("saved 3 figures to pid_plots/")