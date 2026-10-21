import random

from homewrk2.map import MAP_SIZE, find_distance, line_collision
from homewrk2.djikstras import Node

STEP_SIZE = 5.0     # m: fits through the ~7 m narrowest gap
GOAL_TOL = 5.0      # m: matches waypoint acceptance radius
MAX_ITER = 5000     # cap so failed runs terminate


def nearest_node(tree: list, point: tuple) -> int:
    """Index of the tree node closest to point."""
    return min(range(len(tree)), key=lambda i: find_distance((tree[i].x, tree[i].y), point))


def steer(from_pt: tuple, to_pt: tuple, step: float) -> tuple:
    """Move from from_pt toward to_pt by at most `step` meters."""
    d = find_distance(from_pt, to_pt)
    if d <= step:
        return to_pt
    t = step / d
    return (from_pt[0] + t * (to_pt[0] - from_pt[0]),
            from_pt[1] + t * (to_pt[1] - from_pt[1]))


def build_path(tree: list) -> list:
    """Follow parent links from the last node (goal) back to the start."""
    path = []
    i = len(tree) - 1
    while i != -1:
        path.append((tree[i].x, tree[i].y))
        i = tree[i].parent
    path.reverse()
    return path


def rrt(start: tuple, goal: tuple, obstacles: list, seed: int = None,
        step: float = STEP_SIZE, goal_tol: float = GOAL_TOL, max_iter: int = MAX_ITER):
    """Basic RRT in continuous N-E space. Returns (path, tree); path is None on failure."""
    rng = random.Random(seed)   # own random generator, so each seed is reproducible
    tree = [Node(start[0], start[1], 0.0, -1)]

    for _ in range(max_iter):
        sample = (rng.uniform(0, MAP_SIZE), rng.uniform(0, MAP_SIZE))
        near_i = nearest_node(tree, sample)
        near_pt = (tree[near_i].x, tree[near_i].y)
        new_pt = steer(near_pt, sample, step)

        if line_collision(near_pt, new_pt, obstacles):
            continue

        new_cost = tree[near_i].cost + find_distance(near_pt, new_pt)
        tree.append(Node(new_pt[0], new_pt[1], new_cost, near_i))

        # Goal check: close enough and a clear straight shot to the goal
        if find_distance(new_pt, goal) <= goal_tol and not line_collision(new_pt, goal, obstacles):
            tree.append(Node(goal[0], goal[1], new_cost + find_distance(new_pt, goal), len(tree) - 1))
            return build_path(tree), tree

    return None, tree