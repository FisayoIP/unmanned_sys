"""
Problem 1
Usage: python p1_plots.py flight_data.csv
"""

import argparse
import os
import pandas as pd
import matplotlib.pyplot as plt


def get_data(csv):
    return pd.read_csv(csv)


def plot_trajectory(df, outfile="p1_plots/trajectory.png"):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(df["east_m"], df["north_m"], linewidth=1.5)
    ax.scatter(df["east_m"].iloc[0], df["north_m"].iloc[0], c="green", marker="o", label="Start", zorder=5)
    ax.scatter(df["east_m"].iloc[-1], df["north_m"].iloc[-1], c="red", marker="x", label="End", zorder=5)
    ax.set_xlabel("East position, $p_E$ (m)")
    ax.set_ylabel("North position, $p_N$ (m)")
    ax.set_title("Figure 1: North-East Vehicle Trajectory")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True)
    ax.legend()
    fig.text(
        0.5, -0.03,
        "Figure 1. Vehicle ground track in the local NED frame, plotted as North\n"
        "position vs. East position.",
        ha="center", fontsize=9,
    )
    fig.tight_layout()
    fig.savefig(outfile, dpi=200, bbox_inches="tight")
    print(f"Saved {outfile}")


def plot_velocity(df, outfile="p1_plots/velocity.png"):
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(df["t_s"], df["vn_m_s"], label="$V_N$")
    ax.plot(df["t_s"], df["ve_m_s"], label="$V_E$")
    ax.plot(df["t_s"], df["vd_m_s"], label="$V_D$")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Figure 2: North, East, and Down Velocity vs. Time")
    ax.grid(True)
    ax.legend()
    fig.text(
        0.5, -0.05,
        "Figure 2. North, East, and Down components of vehicle velocity\n"
        "over the course of the flight, in the local NED frame.",
        ha="center", fontsize=9,
    )
    fig.tight_layout()
    fig.savefig(outfile, dpi=200, bbox_inches="tight")
    print(f"Saved {outfile}")


def plot_attitude(df, outfile="p1_plots/attitude.png"):
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(df["t_s"], df["roll_deg"], label="Roll ($\\phi$)")
    ax.plot(df["t_s"], df["pitch_deg"], label="Pitch ($\\theta$)")
    ax.plot(df["t_s"], df["yaw_deg"], label="Yaw ($\\psi$)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Figure 3: Roll, Pitch, and Yaw vs. Time")
    ax.grid(True)
    ax.legend()
    fig.text(
        0.5, -0.05,
        "Figure 3. Vehicle roll, pitch, and yaw angles over the course\n"
        "of the flight.",
        ha="center", fontsize=9,
    )
    fig.tight_layout()
    fig.savefig(outfile, dpi=200, bbox_inches="tight")
    print(f"Saved {outfile}")


def main():
    parser = argparse.ArgumentParser(description="Generate HW1 Problem 1(b) plots from the flight-data CSV.")
    parser.add_argument("csv_path", help="Path to the flight data CSV")
    args = parser.parse_args()

    data = get_data(args.csv_path)
    os.makedirs("p1_plots", exist_ok=True)

    plot_trajectory(data)
    plot_velocity(data)
    plot_attitude(data)

    plt.show()


if __name__ == "__main__":
    main()