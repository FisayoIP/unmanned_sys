import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from imports import accel_roll_pitch, gyro_only_integration, complementary_filter, rmse

ALPHA = 0.95

df = pd.read_csv("HOMEWORK_complementary_filter.csv")
os.makedirs("p3_plots", exist_ok=True)

t = df["t_s"].to_numpy()
dt = df["dt_s"].to_numpy()
p = df["gyro_x_rad_s"].to_numpy()
q = df["gyro_y_rad_s"].to_numpy()
ax = df["accel_x_m_s2"].to_numpy()
ay = df["accel_y_m_s2"].to_numpy()
az = df["accel_z_m_s2"].to_numpy()

truth_roll = df["truth_roll_deg"].to_numpy()
truth_pitch = df["truth_pitch_deg"].to_numpy()
ekf_roll = df["ekf_roll_deg"].to_numpy()
ekf_pitch = df["ekf_pitch_deg"].to_numpy()

valid = np.isfinite(truth_roll) & np.isfinite(truth_pitch) & np.isfinite(ekf_roll) & np.isfinite(ekf_pitch)

roll0 = np.deg2rad(truth_roll[valid][0])
pitch0 = np.deg2rad(truth_pitch[valid][0])

roll_acc, pitch_acc = accel_roll_pitch(ax, ay, az)
roll_acc_deg = np.rad2deg(roll_acc)
pitch_acc_deg = np.rad2deg(pitch_acc)

roll_gyro = np.rad2deg(gyro_only_integration(p, dt, roll0))
pitch_gyro = np.rad2deg(gyro_only_integration(q, dt, pitch0))

roll_comp = np.rad2deg(complementary_filter(p, dt, roll_acc, ALPHA, roll0))
pitch_comp = np.rad2deg(complementary_filter(q, dt, pitch_acc, ALPHA, pitch0))

estimators = [
    ("Gyroscope Only", roll_gyro, pitch_gyro),
    ("Accelerometer Only", roll_acc_deg, pitch_acc_deg),
    ("Complementary Filter", roll_comp, pitch_comp),
    ("ArduPilot EKF", ekf_roll, ekf_pitch),
]

rows = []
for name, roll_est, pitch_est in estimators:
    rows.append({
        "Estimator": name,
        "Roll RMSE": rmse(roll_est[valid], truth_roll[valid]),
        "Pitch RMSE": rmse(pitch_est[valid], truth_pitch[valid]),
        "Max Roll Err": np.max(np.abs(roll_est[valid] - truth_roll[valid])),
        "Max Pitch Err": np.max(np.abs(pitch_est[valid] - truth_pitch[valid])),
    })

results = pd.DataFrame(rows)
print(results.round(3).to_string(index=False))

fig, (roll_ax, pitch_ax) = plt.subplots(2, 1, figsize=(11, 9), sharex=True)

roll_ax.plot(t, truth_roll, color="black", linewidth=2.5, label="Truth")
for name, roll_est, _ in estimators:
    roll_ax.plot(t, roll_est, linewidth=1, label=name)
roll_ax.set_ylabel("Roll (deg)")
roll_ax.set_title("Attitude Estimator Comparison: Roll and Pitch vs. Truth")
roll_ax.grid(True)
roll_ax.legend()

pitch_ax.plot(t, truth_pitch, color="black", linewidth=2.5, label="Truth")
for name, _, pitch_est in estimators:
    pitch_ax.plot(t, pitch_est, linewidth=1, label=name)
pitch_ax.set_xlabel("Time (s)")
pitch_ax.set_ylabel("Pitch (deg)")
pitch_ax.grid(True)
pitch_ax.legend()

fig.tight_layout()
fig.savefig("p3_plots/estimator_comparison.png", dpi=200)
plt.show()