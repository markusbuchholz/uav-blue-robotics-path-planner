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


# ------------------------ RRT------------------------------
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

def collision_free_path(node1, node2, grid):
    """Check if the path between node1 and node2 is collision-free."""
    points = np.linspace((node1.x, node1.y, node1.z), (node2.x, node2.y, node2.z), num=100)
    for point in points:
        if grid[int(point[0]), int(point[1]), int(point[2])] == OBSTACLE:
            return False
    return True

def nearest_node(nodes, random_node):
    """Find the nearest node in the tree to the random node."""
    return min(nodes, key=lambda node: np.linalg.norm([node.x - random_node.x, node.y - random_node.y, node.z - random_node.z]))

def steer(nearest, random_node, step_size, grid):
    direction = np.array([random_node.x - nearest.x, random_node.y - nearest.y, random_node.z - nearest.z])
    distance = np.linalg.norm(direction)
    
    if distance == 0:
        return nearest
    
    direction = direction / distance
    new_x = min(max(0, nearest.x + step_size * direction[0]), grid.shape[0]-1)
    new_y = min(max(0, nearest.y + step_size * direction[1]), grid.shape[1]-1)
    new_z = min(max(0, nearest.z + step_size * direction[2]), grid.shape[2]-1)
    
    new_node = Node(new_x, new_y, new_z)
    new_node.parent = nearest
    
    if not collision_free_path(nearest, new_node, grid):
        return None
    
    return new_node

def rrt_3d(start, goal, grid, num_nodes, step_size):
    nodes = [start]
    for _ in range(num_nodes):
        random_node = Node(random.randint(0, grid.shape[0]-1), random.randint(0, grid.shape[1]-1), random.randint(0, grid.shape[2]-1))
        nearest = nearest_node(nodes, random_node)
        new_node = steer(nearest, random_node, step_size, grid)
        if new_node:
            nodes.append(new_node)
            if np.linalg.norm([new_node.x - goal.x, new_node.y - goal.y, new_node.z - goal.z]) < step_size:
                goal.parent = new_node
                nodes.append(goal)
                return nodes
    return nodes

def extract_full_path(goal):
    """Extract the full path from the RRT result."""
    path = []
    current = goal
    while current:
        path.append(current)
        current = current.parent
    return path[::-1]

def simplify_path(full_path):
    """Simplify the path to have only five nodes."""
    total_nodes = len(full_path)
    indices = np.linspace(0, total_nodes-1, 5, dtype=int)
    return [full_path[i] for i in indices]




grid_size = (10, 10, 10)
grid = np.zeros(grid_size)
obstacle = {'corner': (4, 0, 0), 'size': (1, 3, 10)}
add_obstacle_to_grid(obstacle, grid)
    
start = Node(0, 1, 7)
goal = Node(9, 1, 7)
num_nodes = 200
step_size = 1.5

explored_nodes = rrt_3d(start, goal, grid, num_nodes, step_size)
    
# Extract the full path and then simplify it to have only five nodes
full_path = extract_full_path(goal)
simplified_path = simplify_path(full_path)

#-----------------------------------------------
#-----------------------------------------------
#-----------------------------------------------
#-----------------------------------------------
r_path = []
for node in simplified_path:
    #print (node.x, ",", node.y, ",", node.z )
    r_path.append((node.x, node.y, node.z))


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
        delay_loop.poses.append(make_pose(float(r_path[i][0]), float(r_path[i][1]), float(-r_path[i][2])))
        print(float(r_path[i][0]), ",", float(r_path[i][1]), ",", float(-r_path[i][2]))

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
