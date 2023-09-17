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


# ------------------------ Dijkstra------------------------------
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


grid_size = (10, 10, 10)
grid = np.zeros(grid_size)
obstacle = {'corner': (4, 0, 0), 'size': (1, 3, 10)}
add_obstacle_to_grid(obstacle, grid)
    
start = Node(0, 1, 7)
goal = Node(9, 1, 4)

nodes = dijkstra(grid, start, goal)
path_nodes = extract_path(goal, nodes)


r_path = []
i = 0
for node in path_nodes:
    if i > 0:
        r_path.append((node.x, node.y, node.z))
    i = i + 1


#-----------------------------------------------
#-----------------------------------------------

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
