import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
from scipy.spatial import KDTree
from queue import PriorityQueue

OBSTACLE = 1

class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.cost = float('inf')  # For A* search

    def __lt__(self, other):
        return self.cost < other.cost

def add_obstacle_to_grid(obstacle, grid):
    x_start, y_start, z_start = obstacle['corner']
    x_size, y_size, z_size = obstacle['size']
    for x in range(x_start, x_start + x_size):
        for y in range(y_start, y_start + y_size):
            for z in range(z_start, z_start + z_size):
                grid[x, y, z] = OBSTACLE

def is_collision(node, grid):
    return grid[int(node.x), int(node.y), int(node.z)] == OBSTACLE

def collision_free_path(node1, node2, grid):
    """Check if the path between node1 and node2 is collision-free."""
    points = np.linspace((node1.x, node1.y, node1.z), (node2.x, node2.y, node2.z), num=100)
    for point in points:
        if grid[int(point[0]), int(point[1]), int(point[2])] == OBSTACLE:
            return False
    return True

def prm_3d(start, goal, grid, num_samples, k_neighbors):
    nodes = [start, goal]
    for _ in range(num_samples):
        random_node = Node(random.randint(0, grid.shape[0]-1), random.randint(0, grid.shape[1]-1), random.randint(0, grid.shape[2]-1))
        if not is_collision(random_node, grid):
            nodes.append(random_node)

    kdtree = KDTree([(node.x, node.y, node.z) for node in nodes])

    for node in nodes:
        _, indices = kdtree.query([node.x, node.y, node.z], k=k_neighbors)
        for index in indices:
            neighbor = nodes[index]
            if node != neighbor and collision_free_path(node, neighbor, grid):
                if not hasattr(node, 'neighbors'):
                    node.neighbors = []
                node.neighbors.append(neighbor)

    return nodes

def a_star_search(start, goal):
    open_list = PriorityQueue()
    open_list.put(start)
    start.cost = 0
    visited = set()

    while not open_list.empty():
        current = open_list.get()
        if current == goal:
            return True
        visited.add(current)
        for neighbor in current.neighbors:
            if neighbor not in visited:
                tentative_cost = current.cost + np.linalg.norm([neighbor.x - current.x, neighbor.y - current.y, neighbor.z - current.z])
                if tentative_cost < neighbor.cost:
                    neighbor.cost = tentative_cost
                    neighbor.parent = current
                    open_list.put(neighbor)
    return False

def extract_path(goal, grid):
    path = []
    current = goal
    while current:
        path.append(current)
        current = current.parent

    # Prune the path to contain only 6 nodes
    if len(path) > 6:
        step = len(path) // 5
        pruned_path = [path[i] for i in range(0, len(path), step)]
        pruned_path.append(goal)
        pruned_path = pruned_path[:6]

        # Check if pruned path is collision-free
        for i in range(len(pruned_path) - 1):
            if not collision_free_path(pruned_path[i], pruned_path[i+1], grid):
                return path[::-1]  # If pruned path has collision, return the original path
        return pruned_path[::-1]
    return path[::-1]

def plot_3d(nodes, path_nodes, obstacle, start, goal):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plotting the explored nodes
    for node in nodes:
        if hasattr(node, 'neighbors'):
            for neighbor in node.neighbors:
                ax.plot([node.x, neighbor.x], [node.y, neighbor.y], [node.z, neighbor.z], 'k-', alpha=0.3)
    
    # Plotting the final path
    for i in range(len(path_nodes) - 1):
        node = path_nodes[i]
        next_node = path_nodes[i+1]
        ax.plot([node.x, next_node.x], [node.y, next_node.y], [node.z, next_node.z], 'b-')
    
    x_start, y_start, z_start = obstacle['corner']
    x_size, y_size, z_size = obstacle['size']
    ax.bar3d(x_start, y_start, z_start, x_size, y_size, z_size, color='r')
    
    ax.scatter(start.x, start.y, start.z, c='g', s=100, label='Start')
    ax.scatter(goal.x, goal.y, goal.z, c='y', s=100, label='Goal')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.title("PRM path planner")
    plt.show()

if __name__ == "__main__":
    grid_size = (10, 10, 10)
    grid = np.zeros(grid_size)
    obstacle = {'corner': (4, 3, 0), 'size': (1, 3, 10)}
    add_obstacle_to_grid(obstacle, grid)
    
    start = Node(0, 5, 7)
    goal = Node(9, 5, 4)
    num_samples = 100  # Increased the number of samples
    k_neighbors = 10

    nodes = prm_3d(start, goal, grid, num_samples, k_neighbors)
    
    # Search for a path using A* on the PRM graph
    found = a_star_search(start, goal)
    if found:
        path_nodes = extract_path(goal, grid)
        plot_3d(nodes, path_nodes, obstacle, start, goal)
    else:
        print("No path found!")
