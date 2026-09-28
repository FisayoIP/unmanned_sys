"""
ME 459/5559 - Homework 1, Problem 2(b)
Gyro-only attitude estimate: integrate body rates p, q over time and
compare against ground truth to observe drift.
"""

import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt


def gyro_only_integration(rate, dt, theta0=0.0):
    theta = np.zeros(len(rate))
    theta[0] = theta0
    for k in range(1, len(rate)):
        theta[k] = theta[k - 1] + rate[k] * dt[k]
    return np.rad2deg(theta)


def main():
    os.makedirs("gyro_plots", exist_ok=True)
    df = pd.read_csv("HOMEWORK_complementary_filter.csv")

    t = df["t_s"].to_numpy()
    dt = df["dt_s"].to_numpy()
    p = df["gyro_x_rad_s"].to_numpy()   # roll rate
    q = df["gyro_y_rad_s"].to_numpy()   # pitch rate

    truth_roll = df["truth_roll_deg"].to_numpy()
    truth_pitch = df["truth_pitch_deg"].to_numpy()

    # Initialize the estimate at the first available truth value
    # (first couple of rows can be NaN before the truth source kicks in)
    roll0 = truth_roll[np.isfinite(truth_roll)][0]
    pitch0 = truth_pitch[np.isfinite(truth_pitch)][0]

    roll_gyro = gyro_only_integration(p, dt, theta0=np.deg2rad(roll0))
    pitch_gyro = gyro_only_integration(q, dt, theta0=np.deg2rad(pitch0))

    # --- Roll ---
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t, truth_roll, label="Truth", linewidth=2)
    ax.plot(t, roll_gyro, label="Gyro-only estimate", linewidth=1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Roll (deg)")
    ax.set_title("Gyro-Only Roll Estimate vs. Truth")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig("gyro_plots/fig_gyro_only_roll.png", dpi=200)
    print("Saved fig_gyro_only_roll.png")

    # --- Pitch ---
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t, truth_pitch, label="Truth", linewidth=2)
    ax.plot(t, pitch_gyro, label="Gyro-only estimate", linewidth=1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pitch (deg)")
    ax.set_title("Gyro-Only Pitch Estimate vs. Truth")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig("gyro_plots/fig_gyro_only_pitch.png", dpi=200)
    print("Saved fig_gyro_only_pitch.png")

    plt.show()


if __name__ == "__main__":
    main()