visted = dict()
unvisited = dict()

def collision(
    
) -> None:
    ...

def djikstra(
    
) -> None:
    ...
    
class Node:
    def __intit__(self, 
                  x, 
                  y, 
                  cost, 
                  parent
    ) -> None:
        ...

"""
Functions
Distance (wpt1, wpt2)
- calc distance betwen two waypoints

Collision check(
    obstacle list (x,y locations), 
    grid boundaries,
    current node
)
- return true if collision/ bad point

calc_node_idx (
    current node,
    grid boundaries
)
    Return node key/ location index
    
    node_neighbor_cost(
        current node,
        grid info, 
        obstacle list
    )
    return 
    
djisktras(
    start,
    goal, 
    grid, 
    obstacle list
)
"""
min (unvisited, key = lambda x: unvisited[x].cost)