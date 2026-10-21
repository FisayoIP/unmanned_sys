import matplotlib.pyplot as plt
import math

from homewrk2.map import inflate, build_grid, find_distance, desired_angle, line_collision, OBSTACLES, SAFE_RAD, START, GOAL, CRUISE_SPEED
from homewrk2.djikstras import make_grid_info, dijkstra, astar, path_length
from homewrk2.metrics import run_metrics, run_rrt_metrics, run_waypoint_metrics, run_dubins_metrics
from homewrk2.plott import plot_map, plot_search
from homewrk2.metrics import run_metrics, run_rrt_metrics


    
if __name__ == "__main__":
    # Problem 1(a)
    inflated = inflate(obstacles=OBSTACLES, saftey_rad=SAFE_RAD)
    grid = build_grid(inflated)
    print("grid shape:", grid.shape)
    fig, ax = plot_map(grid, OBSTACLES, inflated)
    
    # Problem 1(b) 
    dist = find_distance(START, GOAL)
    course = desired_angle(START, GOAL)
    t_min = dist / CRUISE_SPEED
    print(f"Straight-line distance: {dist:.2f} m")
    print(f"Desired course angle:   {math.degrees(course):.2f} deg  ({course:.4f} rad)")
    print(f"Minimum mission time:   {t_min:.2f} s")
    
    # Problem 1(c) 
    blocked = line_collision(START, GOAL, inflated)
    print(f"Direct route collision free: {not blocked}")
    hits = [i + 1 for i, obs in enumerate(inflated) if line_collision(START, GOAL, [obs])]
    print(f"Direct route intersects obstacles: {hits}")

    # Enables the direct path frmo start to finish
    # ax.plot([START[1], GOAL[1]], [START[0], GOAL[0]], "k--", label="Direct route")
    # ax.legend(loc="lower right")
    
    # Problem 2 
    # See metrics.py to adjusting resolution
    run_metrics(grid, inflated)
    
    # Problem 3 
    rrt_best = run_rrt_metrics(grid, inflated)

    # Problem 4 (a) 
    astar_reduced = run_waypoint_metrics(grid, inflated, rrt_best)
    run_dubins_metrics(grid, inflated, astar_reduced)
    
    
    plt.show()