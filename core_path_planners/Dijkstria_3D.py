import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import heapq

OBSTACLE = 1

class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.distance = float('inf')
        self.visited = False
        self.parent = None

    def __lt__(self, other):
        return self.distance < other.distance

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __hash__(self):
        return hash((self.x, self.y, self.z))

def add_obstacle_to_grid(obstacle, grid):
    x_start, y_start, z_start = obstacle['corner']
    x_size, y_size, z_size = obstacle['size']
    for x in range(x_start, x_start + x_size):
        for y in range(y_start, y_start + y_size):
            for z in range(z_start, z_start + z_size):
                grid[x, y, z] = OBSTACLE

def get_neighbors(node, grid):
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                x, y, z = node.x + dx, node.y + dy, node.z + dz
                if (dx, dy, dz) != (0, 0, 0) and 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1] and 0 <= z < grid.shape[2] and grid[x, y, z] != OBSTACLE:
                    neighbors.append(Node(x, y, z))
    return neighbors

def dijkstra(grid, start, goal):
    nodes = {}
    for x in range(grid.shape[0]):
        for y in range(grid.shape[1]):
            for z in range(grid.shape[2]):
                nodes[(x, y, z)] = Node(x, y, z)

    start_node = nodes[(start.x, start.y, start.z)]
    goal_node = nodes[(goal.x, goal.y, goal.z)]
    start_node.distance = 0
    priority_queue = [start_node]

    while priority_queue:
        current = heapq.heappop(priority_queue)
        if current == goal_node:
            return nodes
        for neighbor in get_neighbors(current, grid):
            neighbor_node = nodes[(neighbor.x, neighbor.y, neighbor.z)]
            if not neighbor_node.visited:
                alt = current.distance + 1
                if alt < neighbor_node.distance:
                    neighbor_node.distance = alt
                    neighbor_node.parent = current
                    heapq.heappush(priority_queue, neighbor_node)
        current.visited = True
    return nodes

def extract_path(goal, nodes):
    path = []
    current = nodes[(goal.x, goal.y, goal.z)]
    while current:
        path.append(current)
        current = current.parent
    return path[::-1]

def plot_3d(path_nodes, obstacle, start, goal):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plotting the final path
    for i in range(len(path_nodes) - 1):
        node = path_nodes[i]
        next_node = path_nodes[i+1]
        ax.plot([node.x, next_node.x], [node.y, next_node.y], [node.z, next_node.z], 'b-')
    
    x_start, y_start, z_start = obstacle['corner']
    x_size, y_size, z_size = obstacle['size']
     ax.bar3d(x_start, y_start, z_start, x_size, y_size, z_size, color=(1, 0, 0, 0.5))
    
    ax.scatter(start.x, start.y, start.z, c='g', s=100, label='Start')
    ax.scatter(goal.x, goal.y, goal.z, c='y', s=100, label='Goal')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.title("Dijkstra path planner")
    plt.show()

if __name__ == "__main__":
    grid_size = (10, 10, 10)
    grid = np.zeros(grid_size)
    obstacle = {'corner': (4, 0, 0), 'size': (1, 3, 10)}
    add_obstacle_to_grid(obstacle, grid)
    
    start = Node(0, 1, 7)
    goal = Node(9, 1, 4)

    nodes = dijkstra(grid, start, goal)
    path_nodes = extract_path(goal, nodes)
    plot_3d(path_nodes, obstacle, start, goal)
