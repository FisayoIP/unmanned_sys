"""
Homework 1, Problem 2(e)

"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def accel_roll_pitch(ax, ay, az):
    roll_acc = np.arctan2(-ay, -az)
    pitch_acc = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
    return roll_acc, pitch_acc


def main():
    df = pd.read_csv("HOMEWORK_complementary_filter.csv")

    ax = df["accel_x_m_s2"].to_numpy()
    ay = df["accel_y_m_s2"].to_numpy()
    az = df["accel_z_m_s2"].to_numpy()
    roll_acc, pitch_acc = accel_roll_pitch(ax, ay, az)
    roll_acc_deg = np.rad2deg(roll_acc)
    pitch_acc_deg = np.rad2deg(pitch_acc)

    t = df["t_s"].to_numpy()
    truth_roll = df["truth_roll_deg"].to_numpy()
    truth_pitch = df["truth_pitch_deg"].to_numpy()

    # the identified high-acceleration maneuver
    t_lo, t_hi = 57.5, 60.5
    mask = (t >= t_lo) & (t <= t_hi)

    fig, (ax_roll, ax_pitch) = plt.subplots(2, 1, figsize=(9, 8), sharex=True)

    ax_roll.plot(t[mask], truth_roll[mask], label="Truth", linewidth=2, color="black")
    ax_roll.plot(t[mask], roll_acc_deg[mask], label="Accelerometer-only", linewidth=1.5, color="tab:red")
    ax_roll.set_ylabel("Roll (deg)")
    ax_roll.set_title("Accelerometer-Only Estimate vs. Truth During High Translational Acceleration")
    ax_roll.grid(True)
    ax_roll.legend()

    ax_pitch.plot(t[mask], truth_pitch[mask], label="Truth", linewidth=2, color="black")
    ax_pitch.plot(t[mask], pitch_acc_deg[mask], label="Accelerometer-only", linewidth=1.5, color="tab:red")
    ax_pitch.set_xlabel("Time (s)")
    ax_pitch.set_ylabel("Pitch (deg)")
    ax_pitch.grid(True)
    ax_pitch.legend()

    fig.tight_layout()
    fig.savefig("fig_part2e_accel_degradation.png", dpi=200)
    print("Saved fig_part2e_accel_degradation.png")
    plt.show()


if __name__ == "__main__":
    main()