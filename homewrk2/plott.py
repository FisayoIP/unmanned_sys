import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle

from homewrk2.map import MAP_SIZE, START, GOAL

def plot_map(grid: np.ndarray, original_obs: list, inflated_obs: list):
    fig, ax = plt.subplots(figsize=(7, 7))

    # Background: inflated occupancy grid (free = white, blocked = light gray)
    cmap = ListedColormap(["white", "lightgray"])
    ax.imshow(grid, origin="lower", extent=[0, MAP_SIZE, 0, MAP_SIZE],
              cmap=cmap, vmin=0, vmax=1)

    # Exact inflated boundaries (dashed) and original obstacles (solid red)
    for i, (nmin, nmax, emin, emax) in enumerate(inflated_obs):
        ax.add_patch(Rectangle((emin, nmin), emax - emin, nmax - nmin,
                               fill=False, edgecolor="gray", linestyle="--", linewidth=1,
                               label="Inflated obstacle (4 m buffer)" if i == 0 else None))
    for i, (nmin, nmax, emin, emax) in enumerate(original_obs):
        ax.add_patch(Rectangle((emin, nmin), emax - emin, nmax - nmin,
                               facecolor="indianred", edgecolor="darkred", alpha=0.8,
                               label="Original obstacle" if i == 0 else None))

    # Start / goal: scatter takes (x=East, y=North)
    ax.scatter(START[1], START[0], c="green", s=90, marker="o", zorder=5, label="Start")
    ax.scatter(GOAL[1], GOAL[0], c="blue", s=140, marker="*", zorder=5, label="Goal")

    # Gridlines every 20 m, drawn under the data
    ticks = np.arange(0, MAP_SIZE + 1, 20)
    ax.set_xticks(ticks)
    ax.set_yticks(ticks)
    ax.grid(True, linewidth=0.5, alpha=0.4)
    ax.set_axisbelow(True)

    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("Planning Environment: Original and Inflated Obstacles")
    ax.set_aspect("equal")
    ax.set_xlim(0, MAP_SIZE)
    ax.set_ylim(0, MAP_SIZE)
    ax.legend(loc="lower right", fontsize=8)
    return fig, ax


def plot_search(grid: np.ndarray, original_obs: list, inflated_obs: list,
                path: list, visited: dict, title: str, color: str = "blue"):
    """Map + expanded nodes + final path. Returns (fig, ax)."""
    fig, ax = plot_map(grid, original_obs, inflated_obs)
    ax.scatter([n.y for n in visited.values()], [n.x for n in visited.values()],
               s=2, c="orange", alpha=0.4, label="Expanded nodes")
    if path is not None:
        ax.plot([p[1] for p in path], [p[0] for p in path], "-", color=color,
                linewidth=2, label="Path")
    ax.set_title(title)
    ax.legend(loc="lower right", fontsize=8, markerscale=6)
    return fig, ax