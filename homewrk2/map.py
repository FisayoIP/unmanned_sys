import math
import numpy as np

MAP_SIZE = 2000
SAFE_RAD = 4
START  = (10,10)
GOAL = (190, 180)
GOAL_RAD = 5
RESOLUTION = 2
OBSTACLES = [(35,70, 25, 90), (85, 120, 75, 130), (130, 165, 20, 85), (145, 180, 120, 165)]


def inflate(obstacles: list, saftey_rad: float):
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

def build_grid(inflated_obstacles: list) -> np.ndarray:
    """Create a 2D occupancy grid. grid[row=N, col=E]: 0 = free, 1 = blocked."""
    # 1. number of cells per side (MAP_SIZE / RESOLUTION, as int)
    cellnum = MAP_SIZE / RESOLUTION
    # 2. blank grid of zeros, that size by that size
    grid = np.zeros((cellnum,cellnum))
    # 3. for each obstacle:
    for obs in inflated_obstacles:
    #      unpack nmin, nmax, emin, emax
        nmin, nmax, emin, emax = obs
    #      low indices  -> met2index (rounds down)
        nmin = 0 if nmin <= 0 else met2index(nmin)
        emin = 0 if emin <= 0 else met2index(emin)
    #      high indices -> math.ceil(value / RESOLUTION) (rounds up, also the slice end)
        nmax = cellnum if nmax >= cellnum else math.ceil(nmax/RESOLUTION)
        emax = cellnum if emax >= cellnum else math.ceil(emax/RESOLUTION)
    #      clamp lows to >= 0 and highs to <= number of cells
    #      fill grid[n_lo:n_hi, e_lo:e_hi] with 1
    grid[nmin:nmax, emin:emax]
    # 4. return grid
    return grid
    



def is_free(grid: np.ndarray, row: int, col: int) -> bool:
    """True if (row, col) is on the map and not blocked."""
    # 1. if row or col is outside 0 to (grid size - 1): return False
    # 2. otherwise return whether grid[row, col] is 0
    ...
    
    
