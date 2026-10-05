"""
ME 459/5559 - Homework 1, Problem 2(c)
Complementary filter for roll and pitch, swept across several alpha values,
with RMSE computed against ground truth for each.
"""

import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt


def accel_roll_pitch(ax, ay, az):
    roll_acc = np.arctan2(-ay, -az)
    pitch_acc = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
    return roll_acc, pitch_acc


def complementary_filter(rate, dt, acc_angle, alpha, theta0):
    theta = np.zeros(len(rate))
    theta[0] = theta0
    for k in range(1, len(rate)):
        gyro_step = theta[k - 1] + rate[k] * dt[k]
        theta[k] = alpha * gyro_step + (1 - alpha) * acc_angle[k]
    return np.rad2deg(theta)


def rmse(estimate, truth):
    return np.sqrt(np.mean((estimate - truth) ** 2))


def main():
    df = pd.read_csv("HOMEWORK_complementary_filter.csv")
    os.makedirs("comp_filter", exist_ok=True)

    dt = df["dt_s"].to_numpy()
    p = df["gyro_x_rad_s"].to_numpy()
    q = df["gyro_y_rad_s"].to_numpy()
    ax = df["accel_x_m_s2"].to_numpy()
    ay = df["accel_y_m_s2"].to_numpy()
    az = df["accel_z_m_s2"].to_numpy()

    truth_roll = df["truth_roll_deg"].to_numpy()
    truth_pitch = df["truth_pitch_deg"].to_numpy()

    roll_acc, pitch_acc = accel_roll_pitch(ax, ay, az)

    roll0 = np.deg2rad(truth_roll[np.isfinite(truth_roll)][0])
    pitch0 = np.deg2rad(truth_pitch[np.isfinite(truth_pitch)][0])

    alphas = [0.50, 0.80, 0.95, 0.98, 0.995]
    results = []

    valid = np.isfinite(truth_roll) & np.isfinite(truth_pitch)

    for a in alphas:
        roll_est = complementary_filter(p, dt, roll_acc, a, roll0)
        pitch_est = complementary_filter(q, dt, pitch_acc, a, pitch0)

        roll_rmse = rmse(roll_est[valid], truth_roll[valid])
        pitch_rmse = rmse(pitch_est[valid], truth_pitch[valid])
        results.append((a, roll_rmse, pitch_rmse))

        # roll, pitch plots
        fig, (ax_roll, ax_pitch) = plt.subplots(2, 1, figsize=(9, 8), sharex=True)

        ax_roll.plot(df["t_s"], truth_roll, label="Truth", linewidth=2, color="black")
        ax_roll.plot(df["t_s"], roll_est, label=f"Complementary (alpha={a})", linewidth=1)
        ax_roll.set_ylabel("Roll (deg)")
        ax_roll.set_title(f"Complementary Filter: Roll and Pitch vs. Truth (alpha={a})")
        ax_roll.grid(True)
        ax_roll.legend()

        ax_pitch.plot(df["t_s"], truth_pitch, label="Truth", linewidth=2, color="black")
        ax_pitch.plot(df["t_s"], pitch_est, label=f"Complementary (alpha={a})", linewidth=1)
        ax_pitch.set_xlabel("Time (s)")
        ax_pitch.set_ylabel("Pitch (deg)")
        ax_pitch.grid(True)
        ax_pitch.legend()

        fig.tight_layout()
        fname = f"comp_filter/comp_alpha_{a}.png"
        fig.savefig(fname, dpi=200)
        print(f"Saved {fname}")

    # Print RMSE table
    print(f"\n{'alpha':>8} | {'Roll RMSE (deg)':>16} | {'Pitch RMSE (deg)':>17}")
    print("-" * 48)
    for a, r_rmse, p_rmse in results:
        print(f"{a:>8.3f} | {r_rmse:>16.3f} | {p_rmse:>17.3f}")

    #plt.show()


if __name__ == "__main__":
    main()