import math

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

def build_grid():
    ...
    
