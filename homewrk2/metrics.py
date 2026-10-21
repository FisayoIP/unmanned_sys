import time
import statistics
import matplotlib.pyplot as plt

from homewrk2.map import OBSTACLES, START, GOAL, CRUISE_SPEED, MIN_TURN_RAD, in_collision
from homewrk2.dubins.dubins_path import dubins_through_waypoints
from homewrk2.djikstras import dijkstra, astar, make_grid_info, path_length
from homewrk2.plott import plot_search, plot_rrt, plot_trimmed, plot_dubins
from homewrk2.waypoints import trim_waypoints
from homewrk2.rrt import rrt

""" Metrics for each Algorithm, Prunins, and Dubins """

def benchmark(algorithm, start, goal, grid_info, obstacles, runs=5):
    times = []
    for _ in range(runs):
        t0 = time.perf_counter()
        path, visited = algorithm(start, goal, grid_info, obstacles)
        times.append(time.perf_counter() - t0)
    return path, visited, statistics.mean(times), statistics.stdev(times)


def run_metrics(grid, inflated):
    for res in [2.0, 1.0]:
        grid_info = make_grid_info(step=res)
        fig, axes = plt.subplots(1, 2, figsize=(13, 7))
        print("\nResolution:", res, "m")

        for i, (name, algorithm) in enumerate([("Dijkstra", dijkstra), ("A*", astar)]):
            path, visited, t_mean, t_std = benchmark(algorithm, START, GOAL, grid_info, inflated)

            print(name)
            if path is None:
                print("  no path found")
            else:
                print("  path length:", round(path_length(path), 2), "m")
                print("  avg time:", round(t_mean * 1000, 1), "ms  (std", round(t_std * 1000, 1), "ms)")
                print("  expanded nodes:", len(visited))
                print("  raw waypoints:", len(path))

            plot_search(grid, OBSTACLES, inflated, path, visited, f"{name} ({res} m grid)", ax=axes[i])

        fig.tight_layout()
  
        
def run_rrt_metrics(grid, inflated):
    lengths = []
    times = []
    tree_sizes = []
    results = []   # (length, seed, path, tree) - internal structure

    print("\nRRT runs")
    for seed in range(10):
        t0 = time.perf_counter()
        path, tree = rrt(START, GOAL, inflated, seed=seed)
        elapsed = time.perf_counter() - t0

        times.append(elapsed)
        tree_sizes.append(len(tree))

        if path is None:
            print("seed", seed, ": failed,", len(tree), "nodes")
            continue

        length = path_length(path)
        lengths.append(length)
        results.append((length, seed, path, tree))
        print("seed", seed, ":", round(length, 2), "m,", round(elapsed * 1000, 1), "ms,", len(tree), "nodes")

    print("\nRRT summary")
    print("success rate:", len(results), "/ 10")
    print("mean path length:", round(statistics.mean(lengths), 2), "m")
    print("std path length:", round(statistics.stdev(lengths), 2), "m")
    print("best path length:", round(min(lengths), 2), "m")
    print("worst path length:", round(max(lengths), 2), "m")
    print("mean time:", round(statistics.mean(times) * 1000, 1), "ms")
    print("mean tree size:", round(statistics.mean(tree_sizes), 1))

    # sort by length: first = best, last = worst, middle = average/typical
    results.sort()
    picks = [("Good", results[0]), ("Poor", results[-1]), ("Typical", results[len(results) // 2])]

    fig, axes = plt.subplots(1, 3, figsize=(19, 7))
    for i in range(3):
        label, (length, seed, path, tree) = picks[i]
        plot_rrt(grid, OBSTACLES, inflated, path, tree,
                 f"RRT {label} (seed {seed}, {length:.1f} m)", ax=axes[i])
    fig.tight_layout()
    
    return results[0][2]


def run_waypoint_metrics(grid, inflated, rrt_best):
    grid_info = make_grid_info(step=2.0)
    dijkstra_path, _ = dijkstra(START, GOAL, grid_info, inflated)
    astar_path, _ = astar(START, GOAL, grid_info, inflated)

    algorithms = [("Dijkstra", dijkstra_path), ("A*", astar_path), ("RRT (best)", rrt_best)]
    fig, axes = plt.subplots(1, 3, figsize=(19, 7))
    reduced_paths = {}

    print("\nWaypoint reduction")
    for i in range(3):
        name, raw = algorithms[i]
        reduced = trim_waypoints(raw, inflated)
        reduced_paths[name] = reduced

        raw_len = path_length(raw)
        red_len = path_length(reduced)
        print(name)
        print(" waypoints:", len(raw), "->", len(reduced))
        print(" length:", round(raw_len, 2), "->", round(red_len, 2), "m")
        print(" est mission time:", round(raw_len / CRUISE_SPEED, 2), "->", round(red_len / CRUISE_SPEED, 2), "s")

        plot_trimmed(grid, OBSTACLES, inflated, raw, reduced, f"{name}: Raw vs Reduced", ax=axes[i])

    fig.tight_layout()
    return reduced_paths["A*"]   


def run_dubins_metrics(grid, inflated, waypoints):
    result = dubins_through_waypoints(waypoints, MIN_TURN_RAD)
    result.collisions = [p for p in result.samples if in_collision(p, inflated)]
    linear_len = path_length(waypoints)

    print("\nDubins (R =", MIN_TURN_RAD, "m)")
    print("  piecewise-linear length:", round(linear_len, 2), "m,",
          round(linear_len / CRUISE_SPEED, 2), "s")
    print("  dubins length:", round(result.length, 2), "m,",
          round(result.length / CRUISE_SPEED, 2), "s")
    print("  segment types:", result.segment_types)
    print("  collision free:", result.collision_free)
    print("  colliding samples:", len(result.collisions))

    plot_dubins(grid, OBSTACLES, inflated, waypoints, result,
                "Reduced A* vs Curvature-Constrained Path")
    return result