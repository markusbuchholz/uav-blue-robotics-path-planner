#!/usr/bin/env python3
from enum import Enum

# """
import rclpy
import rclpy.logging
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from orca_msgs.action import TargetMode
from rclpy.action import ActionClient
from std_msgs.msg import Header
import random


# """
import heapq
import numpy as np
from scipy.spatial import KDTree
from queue import PriorityQueue


# ------------------------ PRM------------------------------
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




grid_size = (10, 10, 10)
grid = np.zeros(grid_size)
obstacle = {'corner': (4, 0, 0), 'size': (1, 3, 10)}
add_obstacle_to_grid(obstacle, grid)
    
start = Node(0, 1, 7)
goal = Node(9, 1, 7)
num_samples = 100  # Increased the number of samples
k_neighbors = 10

nodes = prm_3d(start, goal, grid, num_samples, k_neighbors)
    
    # Search for a path using A* on the PRM graph
r_path = []
i = 0
found = a_star_search(start, goal)
if found:
    path_nodes = extract_path(goal, grid)
    for node in path_nodes:
        if i > 0:
            #print (node.x, ",", node.y, ",", node.z )
            r_path.append((node.x, node.y, node.z))
        i = i + 1
else:
    print("No path found!")


# -----------------------------------------------
# -----------------------------------------------
# -----------------------------------------------
# -----------------------------------------------


class SendGoalResult(Enum):
    SUCCESS = 0     # Goal succeeded
    FAILURE = 1     # Goal failed
    CANCELED = 2    # Goal canceled (KeyboardInterrupt exception)


def make_pose(x: float, y: float, z: float):
    return PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=x, y=y, z=z)))


# Go to AUV mode
go_auv = TargetMode.Goal()
go_auv.target_mode = TargetMode.Goal.ORCA_MODE_AUV

# Go to ROV mode
go_rov = TargetMode.Goal()
go_rov.target_mode = TargetMode.Goal.ORCA_MODE_ROV

# Go home (1m deep)
go_home = FollowWaypoints.Goal()
go_home.poses.append(make_pose(x=0.0, y=0.0, z=-1.0))

# Dive to 8m
dive = FollowWaypoints.Goal()
dive.poses.append(make_pose(x=0.0, y=0.0, z=-8.0))

# Big loop, will eventually result in a loop closure
delay_loop = FollowWaypoints.Goal()
delay_loop.poses.append(make_pose(x=0.0, y=0.0, z=-7.0))


################################################
################################################
for i in range(len(r_path)):
    if i > 0:
        delay_loop.poses.append(
            make_pose(float(r_path[i][0]), float(r_path[i][1]), float(-r_path[i][2])))
        print(float(r_path[i][0]), ",", float(
            r_path[i][1]), ",", float(-r_path[i][2]))

################################################
################################################


# Send a goal to an action server and wait for the result.
# Cancel the goal if the user hits ^C (KeyboardInterrupt).
def send_goal(node, action_client, send_goal_msg) -> SendGoalResult:
    goal_handle = None

    try:
        action_client.wait_for_server()

        print('Sending goal...')
        goal_future = action_client.send_goal_async(send_goal_msg)
        rclpy.spin_until_future_complete(node, goal_future)
        goal_handle = goal_future.result()

        if goal_handle is None:
            raise RuntimeError(
                'Exception while sending goal: {!r}'.format(goal_future.exception()))

        if not goal_handle.accepted:
            print('Goal rejected')
            return SendGoalResult.FAILURE

        print('Goal accepted with ID: {}'.format(
            bytes(goal_handle.goal_id.uuid).hex()))
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)

        result = result_future.result()

        if result is None:
            raise RuntimeError('Exception while getting result: {!r}'.format(
                result_future.exception()))

        print('Goal completed')
        return SendGoalResult.SUCCESS

    except KeyboardInterrupt:
        # Cancel the goal if it's still active
        # TODO(clyde): this seems to work, but a second exception is generated -- why?
        if (goal_handle is not None and
                (GoalStatus.STATUS_ACCEPTED == goal_handle.status or
                 GoalStatus.STATUS_EXECUTING == goal_handle.status)):
            print('Canceling goal...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future)
            cancel_response = cancel_future.result()

            if cancel_response is None:
                raise RuntimeError('Exception while canceling goal: {!r}'.format(
                    cancel_future.exception()))

            if len(cancel_response.goals_canceling) == 0:
                raise RuntimeError('Failed to cancel goal')
            if len(cancel_response.goals_canceling) > 1:
                raise RuntimeError('More than one goal canceled')
            if cancel_response.goals_canceling[0].goal_id != goal_handle.goal_id:
                raise RuntimeError('Canceled goal with incorrect goal ID')

            print('Goal canceled')
            return SendGoalResult.CANCELED


def main():
    node = None
    set_target_mode = None
    follow_waypoints = None

    rclpy.init()

    try:
        node = rclpy.create_node("mission_runner")

        set_target_mode = ActionClient(node, TargetMode, '/set_target_mode')
        follow_waypoints = ActionClient(
            node, FollowWaypoints, '/follow_waypoints')

        print('>>> Setting mode to AUV <<<')
        if send_goal(node, set_target_mode, go_auv) == SendGoalResult.SUCCESS:
            print('>>> Executing mission <<<')
            send_goal(node, follow_waypoints, delay_loop)

            print('>>> Setting mode to ROV <<<')
            send_goal(node, set_target_mode, go_rov)

            print('>>> Mission complete <<<')
        else:
            print('>>> Failed to set mode to AUV, quit <<<')

    finally:
        if set_target_mode is not None:
            set_target_mode.destroy()
        if follow_waypoints is not None:
            follow_waypoints.destroy()
        if node is not None:
            node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
# """
