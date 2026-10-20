import math
import numpy as np

MAP_SIZE = 200
SAFE_RAD = 4
CRUISE_SPEED = 12 #m/s
START  = (10,10)
GOAL = (190, 180)
GOAL_RAD = 5
RESOLUTION = 2
OBSTACLES = [(35,70, 25, 90), 
             (85, 120, 75, 130), 
             (130, 165, 20, 85), 
             (145, 180, 120, 165)]


def inflate(obstacles=OBSTACLES, saftey_rad=SAFE_RAD):
    new = []
    for coords in obstacles:
        nmin, nmax, emin, emax = coords
        new.append((nmin-saftey_rad, nmax+saftey_rad, emin-saftey_rad, emax+saftey_rad))
    return new   

def find_distance(p1: tuple, p2: tuple):
    n1,e1 = p1
    n2,e2 = p2
    deltaN = n2-n1
    deltaE = e2-e1
    dist = math.sqrt((deltaN)**2 + deltaE**2)
    return dist
   
def desired_angle(p1: tuple, p2: tuple):
    n1,e1 = p1
    n2,e2 = p2
    deltaN = n2-n1
    deltaE = e2-e1
    return math.atan2(deltaE, deltaN)

def met2index(meters: float) -> int:
    return int((meters//RESOLUTION))

def index2met(idx: int) -> float:
    return (idx*RESOLUTION) + (RESOLUTION/2)

def build_grid(obstacles: list) -> np.ndarray:
    # Create a 2D occupancy grid. grid[row=N, col=E]: 0 = free, 1 = blocked.
    cellnum = int(MAP_SIZE / RESOLUTION)
    grid = np.zeros((cellnum,cellnum), dtype=int)
    for obs in obstacles:
        nmin, nmax, emin, emax = obs
        n_low = 0 if nmin <= 0 else met2index(nmin)
        e_low = 0 if emin <= 0 else met2index(emin)
        n_max = cellnum if nmax >= MAP_SIZE else math.ceil(nmax/RESOLUTION)
        e_max = cellnum if emax >= MAP_SIZE else math.ceil(emax/RESOLUTION)
        grid[n_low:n_max, e_low:e_max] = 1
    return grid
    

def in_collision(p: tuple, obstacles: list) -> bool:
    # True if point p = (N, E) is off the map or inside any obstacle.
    n, e = p
    if not (0 <= n <= MAP_SIZE and 0 <= e <= MAP_SIZE):
        return True
    for nmin, nmax, emin, emax in obstacles:
        if nmin <= n <= nmax and emin <= e <= emax:
            return True
    return False

def line_collision(p1: tuple, p2: tuple, obstacles: list, step: float = 0.5) -> bool:
    # True if the straight line from p1 to p2 hits an obstacle.
    n_steps = max(1, math.ceil(find_distance(p1, p2) / step))
    for k in range(n_steps + 1):
        t = k / n_steps            # 0 at p1, 1 at p2
        point = (p1[0] + t * (p2[0] - p1[0]),
                 p1[1] + t * (p2[1] - p1[1]))
        if in_collision(point, obstacles):
            return True
    return False

