import matplotlib.pyplot as plt
import math

from homewrk2.map import inflate, build_grid, find_distance, desired_angle, line_collision, OBSTACLES, SAFE_RAD, START, GOAL, CRUISE_SPEED
from homewrk2.djikstras import make_grid_info, dijkstra, astar, path_length
from homewrk2.plott import plot_map, plot_search

    
if __name__ == "__main__":
    # --- Problem 1(a) ---
    inflated = inflate(obstacles=OBSTACLES, saftey_rad=SAFE_RAD)
    grid = build_grid(inflated)
    print("grid shape:", grid.shape)
    fig, ax = plot_map(grid, OBSTACLES, inflated)
    
    # --- Problem 1(b) ---
    dist = find_distance(START, GOAL)
    course = desired_angle(START, GOAL)
    t_min = dist / CRUISE_SPEED
    print(f"Straight-line distance: {dist:.2f} m")
    print(f"Desired course angle:   {math.degrees(course):.2f} deg  ({course:.4f} rad)")
    print(f"Minimum mission time:   {t_min:.2f} s")
    
    # --- Problem 1(c) ---
    blocked = line_collision(START, GOAL, inflated)
    print(f"Direct route collision free: {not blocked}")
    hits = [i + 1 for i, obs in enumerate(inflated) if line_collision(START, GOAL, [obs])]
    print(f"Direct route intersects obstacles: {hits}")

    # ax.plot([START[1], GOAL[1]], [START[0], GOAL[0]], "k--", label="Direct route")
    # ax.legend(loc="lower right")
    
    
    # --- Problem 2(a) ---
    grid_info = make_grid_info()
    path, visited = dijkstra(START, GOAL, grid_info, inflated)

    if len(path) == 0:
        print("Dijkstra: no path found")
    else:
        print(f"Dijkstra path length: {path_length(path):.2f} m")
        print(f"Expanded nodes: {len(visited)}")
        print(f"Raw waypoints: {len(path)}")

        fig2, ax2 = plot_map(grid, OBSTACLES, inflated)
        ax2.scatter([n.y for n in visited.values()], [n.x for n in visited.values()],
                    s=2, c="orange", alpha=0.4, label="Expanded nodes")
        ax2.plot([p[1] for p in path], [p[0] for p in path], "b-", linewidth=2, label="Dijkstra path")
        ax2.set_title("Dijkstra: Path and Explored Nodes")
        ax2.legend(loc="lower right", fontsize=8)
        
        # --- Problem 2(a) and 2(b) ---
    grid_info = make_grid_info()
    for name, planner in [("Dijkstra", dijkstra), ("A*", astar)]:
        path, visited = planner(START, GOAL, grid_info, inflated)
        if path is None:
            print(f"{name}: no path found")
            continue
        print(f"{name}: length {path_length(path):.2f} m | "
              f"expanded {len(visited)} | raw waypoints {len(path)}")
        plot_search(grid, OBSTACLES, inflated, path, visited, f"{name}: Path and Explored Nodes")
        
    plt.show()