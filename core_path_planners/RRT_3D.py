import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random

OBSTACLE = 1

class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None

def add_obstacle_to_grid(obstacle, grid):
    x_start, y_start, z_start = obstacle['corner']
    x_size, y_size, z_size = obstacle['size']
    for x in range(x_start, x_start + x_size):
        for y in range(y_start, y_start + y_size):
            for z in range(z_start, z_start + z_size):
                grid[x, y, z] = OBSTACLE

def is_collision(node, grid):
    return grid[int(node.x), int(node.y), int(node.z)] == OBSTACLE

def is_edge_collision(node1, node2, grid):
    """Check if the edge between node1 and node2 intersects with an obstacle."""
    points = np.linspace((node1.x, node1.y, node1.z), (node2.x, node2.y, node2.z), num=10)
    for point in points:
        if grid[int(point[0]), int(point[1]), int(point[2])] == OBSTACLE:
            return True
    return False

def steer(nearest, random_node, step_size, grid):
    direction = np.array([random_node.x - nearest.x, random_node.y - nearest.y, random_node.z - nearest.z])
    distance = np.linalg.norm(direction)
    
    # Check for zero vector
    if distance == 0:
        return nearest
    
    direction = direction / distance
    new_x = min(max(0, nearest.x + step_size * direction[0]), grid.shape[0]-1)
    new_y = min(max(0, nearest.y + step_size * direction[1]), grid.shape[1]-1)
    new_z = min(max(0, nearest.z + step_size * direction[2]), grid.shape[2]-1)
    
    new_node = Node(new_x, new_y, new_z)
    new_node.parent = nearest
    
    if is_edge_collision(nearest, new_node, grid):
        return None
    
    return new_node

def nearest_node(nodes, random_node):
    """Find the nearest node in the tree to the random node."""
    return min(nodes, key=lambda node: np.linalg.norm([node.x - random_node.x, node.y - random_node.y, node.z - random_node.z]))


def rrt_3d(start, goal, grid, num_nodes, step_size):
    nodes = [start]
    for _ in range(num_nodes):
        random_node = Node(random.randint(0, grid.shape[0]-1), random.randint(0, grid.shape[1]-1), random.randint(0, grid.shape[2]-1))
        nearest = nearest_node(nodes, random_node)
        new_node = steer(nearest, random_node, step_size, grid)
        if new_node and not is_collision(new_node, grid):
            nodes.append(new_node)
            if np.linalg.norm([new_node.x - goal.x, new_node.y - goal.y, new_node.z - goal.z]) < step_size:
                goal.parent = new_node
                nodes.append(goal)
                return nodes
    return nodes


def plot_3d(nodes, obstacle, start, goal):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
      for node in nodes:
        if node.parent:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], [node.z, node.parent.z], 'b-')
    
      x_start, y_start, z_start = obstacle['corner']
    x_size, y_size, z_size = obstacle['size']
    ax.bar3d(x_start, y_start, z_start, x_size, y_size, z_size, color=(1, 0, 0, 0.5))
    
    ax.scatter(start.x, start.y, start.z, c='g', s=100, label='Start')
    ax.scatter(goal.x, goal.y, goal.z, c='y', s=100, label='Goal')
    
     path_node = goal
    while path_node.parent:
        ax.plot([path_node.x, path_node.parent.x], [path_node.y, path_node.parent.y], [path_node.z, path_node.parent.z], 'g-', linewidth=2.5)
        path_node = path_node.parent
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.title("RRT path planner")
    plt.show()


if __name__ == "__main__":
    grid_size = (10, 10, 10)
    grid = np.zeros(grid_size)
    obstacle = {'corner': (4, 3, 0), 'size': (1, 3, 10)}
    add_obstacle_to_grid(obstacle, grid)
    
    start = Node(0, 5, 7)
    goal = Node(9, 5, 7)
    num_nodes = 1000
    step_size = 1.5

    nodes = rrt_3d(start, goal, grid, num_nodes, step_size)
    plot_3d(nodes, obstacle, start, goal)

