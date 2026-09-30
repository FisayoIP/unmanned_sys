import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from imports import accel_roll_pitch, gyro_only_integration, complementary_filter, rmse

ALPHA = 0.95
BIAS_DEG_S = .5

df = pd.read_csv("HOMEWORK_complementary_filter.csv")
os.makedirs("p3_plots", exist_ok=True)

t = df["t_s"].to_numpy()
dt = df["dt_s"].to_numpy()
p = df["gyro_x_rad_s"].to_numpy()
ax = df["accel_x_m_s2"].to_numpy()
ay = df["accel_y_m_s2"].to_numpy()
az = df["accel_z_m_s2"].to_numpy()
truth_roll = df["truth_roll_deg"].to_numpy()

valid = np.isfinite(truth_roll)
roll0 = np.deg2rad(truth_roll[valid][0])

roll_acc, _ = accel_roll_pitch(ax, ay, az)

bias_rad_s = np.deg2rad(BIAS_DEG_S)
p_biased = p + bias_rad_s

roll_gyro_base = np.rad2deg(gyro_only_integration(p, dt, roll0))
roll_gyro_biased = np.rad2deg(gyro_only_integration(p_biased, dt, roll0))

roll_comp_base = np.rad2deg(complementary_filter(p, dt, roll_acc, ALPHA, roll0))
roll_comp_biased = np.rad2deg(complementary_filter(p_biased, dt, roll_acc, ALPHA, roll0))

runs = [
    ("Gyro-only, no bias", roll_gyro_base),
    (f"Gyro-only, +{BIAS_DEG_S} deg/s bias", roll_gyro_biased),
    ("Complementary, no bias", roll_comp_base),
    (f"Complementary, +{BIAS_DEG_S} deg/s bias", roll_comp_biased),
]

rows = []
for name, est in runs:
    rows.append({
        "Run": name,
        "Roll RMSE": rmse(est[valid], truth_roll[valid]),
        "Max Roll Err": np.max(np.abs(est[valid] - truth_roll[valid])),
    })

results = pd.DataFrame(rows)
print(results.round(3).to_string(index=False))
results.to_csv("p3_plots/gyro_bias_results.csv", index=False)

err = np.abs(roll_gyro_biased[valid] - truth_roll[valid])
t_valid = t[valid]
over_10 = np.where(err > 10.0)[0]
if len(over_10) > 0:
    print(f"\nBiased gyro-only exceeds 10 deg error at t = {t_valid[over_10[0]]:.2f} s")

fig, (top, bottom) = plt.subplots(2, 1, figsize=(11, 9), sharex=True)

top.plot(t, truth_roll, color="black", linewidth=2.5, label="Truth")
top.plot(t, roll_gyro_base, linewidth=1, linestyle="--", label="Gyro-only (no bias)")
top.plot(t, roll_gyro_biased, linewidth=1.5, color="tab:red", label="Gyro-only (+0.5 deg/s bias)")
top.set_ylabel("Roll (deg)")
top.set_title("Effect of Constant Gyro Bias on Roll Estimates")
top.grid(True)
top.legend()

bottom.plot(t, truth_roll, color="black", linewidth=2.5, label="Truth")
bottom.plot(t, roll_comp_base, linewidth=1, linestyle="--", label="Complementary (no bias)")
bottom.plot(t, roll_comp_biased, linewidth=1.5, color="tab:orange", label="Complementary (+0.5 deg/s bias)")
bottom.set_xlabel("Time (s)")
bottom.set_ylabel("Roll (deg)")
bottom.grid(True)
bottom.legend()

fig.tight_layout()
fig.savefig("p3_plots/gyro_bias_effect.png", dpi=200)
plt.show()