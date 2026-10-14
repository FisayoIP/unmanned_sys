import heapq
import matplotlib.pyplot as plt
import numpy as np

# 1. Define the grid size and costs (1 for normal cells, inf/high for obstacles)
rows, cols = 10, 10
grid = np.ones((rows, cols), dtype=float)

# Add some obstacles (e.g., set to infinity or a very high number)
grid[4, 1:8] = np.inf
grid[2:7, 5] = np.inf

start = (0, 0)
end = (9, 9)


def dijkstra(grid, start, end):
  rows, cols = grid.shape
  distances = np.full((rows, cols), np.inf)
  distances[start] = grid[start]
  parent = {}
  pq = [(distances[start], start)]

  while pq:
    current_dist, (r, c) = heapq.heappop(pq)

    if (r, c) == end:
      break

    if current_dist > distances[r, c]:
      continue

    # 4-connected neighbors
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
      nr, nc = r + dr, c + dc
      if 0 <= nr < rows and 0 <= nc < cols:
        weight = grid[nr, nc]
        if np.isinf(weight):
          continue
        new_dist = current_dist + weight
        if new_dist < distances[nr, nc]:
          distances[nr, nc] = new_dist
          parent[(nr, nc)] = (r, c)
          heapq.heappush(pq, (new_dist, (nr, nc)))

  return distances, parent


distances, parent = dijkstra(grid, start, end)

# Reconstruct path
path = []
curr = end
while curr in parent:
  path.append(curr)
  curr = parent[curr]
path.append(start)
path.reverse()

# 2. Visualize with Matplotlib
fig, ax = plt.subplots(figsize=(6, 6))

# Plot grid background (obstacles as dark, open space as light)
display_grid = np.copy(grid)
display_grid[np.isinf(display_grid)] = -1  # mark walls
ax.imshow(display_grid, cmap='cividis', origin='upper')

# Extract path coordinates
if len(path) > 1:
  pr, pc = zip(*path)
  ax.plot(pc, pr, color='red', linewidth=2, label='Shortest Path')

ax.scatter(start[1], start[0], color='lime', s=100, label='Start')
ax.scatter(end[1], end[0], color='cyan', s=100, label='End')
ax.grid(
    which='both', color='gray', linestyle='-', linewidth=0.5
)  
ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
ax.legend()
plt.title("Dijkstra's Shortest Path on a Grid")
plt.show()
