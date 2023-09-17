import numpy as np
import heapq
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the 3D grid
grid_size = 10
grid = np.zeros((grid_size, grid_size, grid_size))

# Define start and goal
start = (0, 5, 7)
goal = (9, 5, 7)

#start = (0, 5, 5)
#goal = (10, 5, 5)

# Define obstacles as cubes
# Each obstacle is defined by its lower corner and its size
# obstacle1 = {'corner': (4, 4, 4), 'size': 3}



# for x in range(obstacle1['corner'][0], obstacle1['corner'][0] + obstacle1['size']):
#     for y in range(obstacle1['corner'][1], obstacle1['corner'][1] + obstacle1['size']):
#         for z in range(obstacle1['corner'][2], obstacle1['corner'][2] + obstacle1['size']):
#             grid[x, y, z] = 1
# Define obstacle of size 2x1x7
obstacle = {'corner': (4, 3, 0), 'size': (1, 3, 10)}
OBSTACLE = 1
def add_obstacle_to_grid(obstacle, grid):
    x_start, y_start, z_start = obstacle['corner']
    x_size, y_size, z_size = obstacle['size']
    for x in range(x_start, x_start + x_size):
        for y in range(y_start, y_start + y_size):
            for z in range(z_start, z_start + z_size):
                grid[x, y, z] = OBSTACLE

add_obstacle_to_grid(obstacle, grid)

# Define the heuristic function
def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

# Define the A* algorithm
def astar(array, start, goal):
    neighbors = [(0,1,0),(1,0,0),(0,-1,0),(-1,0,0),(0,0,1),(0,0,-1)]
    
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    open_heap = []

    heapq.heappush(open_heap, (fscore[start], start))
    
    while open_heap:
        current = heapq.heappop(open_heap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j, k in neighbors:
            neighbor = current[0] + i, current[1] + j, current[2] + k            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if 0 <= neighbor[2] < array.shape[2]:
                        if array[neighbor[0]][neighbor[1]][neighbor[2]] == 1:
                            continue
                    else:
                        continue
                else:
                    continue
            else:
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in open_heap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (fscore[neighbor], neighbor))
                
    return False

path = astar(grid, start, goal)
r_path = []


# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the path as a line
if path:
    path = [start] + path[::-1]  # Add the start to the path and reverse it
    xs, ys, zs = zip(*path)
    for i in range (len(xs)):
        print(xs[i],ys[i],zs[i])
        r_path.insert(0, (xs[i],ys[i],zs[i]))
    ax.plot(xs, ys, zs, color="r", linewidth=2)
    print ("-------------------------------")
    #xsr, ysr, zsr = zip(*reversed_path)
    for i in range (len(r_path)):
        print(r_path[i][0], r_path[i][1],r_path[i][2])
     #   print(xsr[i],ysr[i],zsr[i])

# Plot start and goal
ax.scatter(*start, color="g", s=100, label="Start")
ax.scatter(*goal, color="y", s=100, label="Goal")

# Plot obstacles as cubes
x, y, z = np.indices((grid_size+1, grid_size+1, grid_size+1))
ax.voxels(x, y, z, grid, color="b", edgecolor="k")

ax.legend()
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
plt.title('A* path planner')
plt.show()

