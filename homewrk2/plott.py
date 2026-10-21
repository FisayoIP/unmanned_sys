import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle

from homewrk2.map import MAP_SIZE, START, GOAL

def plot_map(grid: np.ndarray, original_obs: list, inflated_obs: list, ax=None):
    if ax is None:
        fig, ax = plt.subplots(figsize=(7, 7))
    else:
        fig = ax.figure
        
    cmap = ListedColormap(["white", "lightgray"])
    ax.imshow(grid, origin="lower", extent=[0, MAP_SIZE, 0, MAP_SIZE],
              cmap=cmap, vmin=0, vmax=1)

    for i, (nmin, nmax, emin, emax) in enumerate(inflated_obs):
        ax.add_patch(Rectangle((emin, nmin), emax - emin, nmax - nmin,
                               fill=False, edgecolor="gray", linestyle="--", linewidth=1,
                               label="Inflated obstacle (4 m buffer)" if i == 0 else None))
    for i, (nmin, nmax, emin, emax) in enumerate(original_obs):
        ax.add_patch(Rectangle((emin, nmin), emax - emin, nmax - nmin,
                               facecolor="indianred", edgecolor="darkred", alpha=0.8,
                               label="Original obstacle" if i == 0 else None))

    # Start / goal
    ax.scatter(START[1], START[0], c="green", s=90, marker="o", zorder=5, label="Start")
    ax.scatter(GOAL[1], GOAL[0], c="blue", s=140, marker="*", zorder=5, label="Goal")

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
                path: list, visited: dict, title: str, color: str = "blue", ax=None):
    fig, ax = plot_map(grid, original_obs, inflated_obs, ax=ax)
    ax.scatter([n.y for n in visited.values()], [n.x for n in visited.values()],
               s=2, c="orange", alpha=0.4)                              
    ax.scatter([], [], s=20, c="orange", label="Expanded nodes")          
    if path is not None:
        ax.plot([p[1] for p in path], [p[0] for p in path], "-", color=color,
                linewidth=2, label="Path")
    ax.set_title(title)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=3, fontsize=8)
    return fig, ax


def plot_rrt(grid: np.ndarray, original_obs: list, inflated_obs: list,
             path: list, tree: list, title: str, ax=None):
    fig, ax = plot_map(grid, original_obs, inflated_obs, ax=ax)
    for node in tree:
        if node.parent != -1:
            parent = tree[node.parent]
            ax.plot([parent.y, node.y], [parent.x, node.x], color="orange", linewidth=0.5, alpha=0.6)
    ax.plot([], [], color="orange", label="RRT tree")  
    if path is not None:
        ax.plot([p[1] for p in path], [p[0] for p in path], "b-", linewidth=2, label="Path")
    ax.set_title(title)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=3, fontsize=8)
    return fig, ax


def plot_trimmed(grid, original_obs, inflated_obs, raw, reduced, title, ax=None):
    fig, ax = plot_map(grid, original_obs, inflated_obs, ax=ax)
    ax.plot([p[1] for p in raw], [p[0] for p in raw], color="orange", linewidth=1,
            label=f"Raw path ({len(raw)} pts)")
    ax.plot([p[1] for p in reduced], [p[0] for p in reduced], "b-o", linewidth=2, markersize=6,
            label=f"Reduced ({len(reduced)} pts)")
    ax.set_title(title)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=3, fontsize=8)
    return fig, ax


def plot_dubins(grid, original_obs, inflated_obs, waypoints, result, title, ax=None):
    fig, ax = plot_map(grid, original_obs, inflated_obs, ax=ax)
    ax.plot([p[1] for p in waypoints], [p[0] for p in waypoints], "k--o", linewidth=1,
            markersize=5, label="Reduced A* (piecewise linear)")
    ax.plot([p[1] for p in result.samples], [p[0] for p in result.samples], "b-", linewidth=2,
            label="Dubins path (R = 20 m)")
    if result.collisions:
        ax.scatter([p[1] for p in result.collisions], [p[0] for p in result.collisions],
                   c="red", marker="x", s=25, zorder=6, label="Collision")
    ax.set_title(title)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2, fontsize=8)
    return fig, ax