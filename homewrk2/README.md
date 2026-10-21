
# ME 459 Homework 2: Guidance, Mission Planning, and Path Planning

A quick how-to for the files in this folder. Should be a pretty streamlined

## Setup

All you need is uv sync, everything else is imported


## Running

Main entry point is `homewrk2/main.py`. Used to generate graphs for all questions

```
uv run python -m homewrk2.main
```


## Files

| File | Purpose |
|---|---|
| `main.py` | Start Point - Generates all graphs for the assignment |
| `map.py` | Parameters, obstacle inflation, occupancy grid, distance, course angle, collision checks |
| `djikstras.py` | A `dijkstra()` function atthe core with an `astar()` flag |
| `rrt.py` | RRT algorithm with the requested requirements |
| `waypoints.py` | Line-of-sight waypoint reduction |
| `metrics.py` | Generates metrics for each graph |
| `plott.py` | All plotting functions |
| `dubins/` | Dubins folder provided by professor |
| `test/test_map.py` | A few unit tests |

## Run the tests

```
uv run pytest homewrk2/test/test_map.py -v
```