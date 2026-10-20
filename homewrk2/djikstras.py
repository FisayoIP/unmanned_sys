import math

from homewrk2.map import MAP_SIZE, RESOLUTION, in_collision, find_distance


class Node:
    def __init__(self, x: float, y: float, cost: float, parent: int, h: float = 0.0):
        self.x = x            # North (m)
        self.y = y            # East (m)
        self.cost = cost      # cost to reach this node from start (m)
        self.parent = parent  # index of the node we came from; -1 = start
        self.h = h            # heuristic estimate to goal (0 for Dijkstra)


def make_grid_info(min_x: float = 0.0, max_x: float = MAP_SIZE,
                   min_y: float = 0.0, max_y: float = MAP_SIZE,
                   step: float = RESOLUTION) -> dict:
    """Bundle the search-grid boundaries and step size."""
    width = int(round((max_y - min_y) / step)) + 1   # number of East positions per row
    return {"min_x": min_x, "max_x": max_x, "min_y": min_y, "max_y": max_y,
            "step": step, "width": width}


def calc_node_idx(x: float, y: float, grid: dict) -> int:
    """Unique integer key for a position: row * width + col."""
    row = int(round((x - grid["min_x"]) / grid["step"]))
    col = int(round((y - grid["min_y"]) / grid["step"]))
    return row * grid["width"] + col


def is_valid(node: Node, obstacles: list) -> bool:
    """True if the node is on the map and outside every inflated obstacle."""
    return not in_collision((node.x, node.y), obstacles)


def graph_search(start: tuple, goal: tuple, grid: dict, obstacles: list, use_heuristic: bool):
    """Shared search on an 8-connected grid.
    use_heuristic=False -> Dijkstra (picks lowest cost)
    use_heuristic=True  -> A* (picks lowest cost + straight-line distance to goal)
    Returns (path, visited); path is None on failure."""
    step = grid["step"]
    visited = {}
    unvisited = {}

    def h(x, y):
        return find_distance((x, y), goal) if use_heuristic else 0.0

    unvisited[calc_node_idx(start[0], start[1], grid)] = Node(start[0], start[1], 0.0, -1, h(*start))
    goal_idx = calc_node_idx(goal[0], goal[1], grid)

    while unvisited:
        # The only line that differs between Dijkstra and A*: h is 0 for Dijkstra
        curr_idx = min(unvisited, key=lambda k: unvisited[k].cost + unvisited[k].h)
        current = unvisited.pop(curr_idx)
        visited[curr_idx] = current

        if curr_idx == goal_idx:
            return find_shortest_path(goal, visited, grid), visited

        # 8 neighbors: every (dx, dy) combo of -step, 0, +step except (0, 0)
        for dx in (-step, 0, step):
            for dy in (-step, 0, step):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = current.x + dx, current.y + dy
                temp = Node(nx, ny, current.cost + math.hypot(dx, dy), curr_idx, h(nx, ny))
                temp_idx = calc_node_idx(nx, ny, grid)

                if temp_idx in visited or not is_valid(temp, obstacles):
                    continue
                if temp_idx in unvisited:
                    if temp.cost < unvisited[temp_idx].cost:
                        unvisited[temp_idx].cost = temp.cost
                        unvisited[temp_idx].parent = curr_idx
                else:
                    unvisited[temp_idx] = temp

    return None, visited   # ran out of nodes without reaching the goal


def dijkstra(start: tuple, goal: tuple, grid: dict, obstacles: list):
    return graph_search(start, goal, grid, obstacles, use_heuristic=False)

def astar(start: tuple, goal: tuple, grid: dict, obstacles: list):
    return graph_search(start, goal, grid, obstacles, use_heuristic=True)


def find_shortest_path(goal: tuple, visited: dict, grid: dict) -> list:
    """Follow parent links from the goal back to the start. Returns [(N, E), ...] start -> goal."""
    path = [goal]
    node_idx = calc_node_idx(goal[0], goal[1], grid)
    while visited[node_idx].parent != -1:
        node_idx = visited[node_idx].parent
        node = visited[node_idx]
        path.append((node.x, node.y))
    path.reverse()
    return path


def path_length(path: list) -> float:
    """Total length of a path in meters."""
    return sum(find_distance(path[i], path[i + 1]) for i in range(len(path) - 1))