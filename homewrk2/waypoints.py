from homewrk2.map import line_collision


def trim_waypoints(path: list, obstacles: list) -> list:
    """LOS Waypoint Reduction"""
    """From the current waypoint, jump to the furthest later waypoint with a clear straight line."""
    if path is None or len(path) < 3:
        return path

    reduced = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and line_collision(path[i], path[j], obstacles):
            j -= 1
        reduced.append(path[j])
        i = j
    return reduced