#!/usr/bin/env python3


from enum import Enum

#"""
import rclpy
import rclpy.logging
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from orca_msgs.action import TargetMode
from rclpy.action import ActionClient
from std_msgs.msg import Header

#"""
import heapq
import numpy as np


# ------------------------ A*------------------------------

# Define the 3D grid
grid_size = 10
grid = np.zeros((grid_size, grid_size, grid_size))
costs = np.random.randint(1, 11, (grid_size, grid_size, grid_size))

# Define start and goal

start = (0, 1, 7)
goal = (9, 1, 7)


obstacle = {'corner': (4, 0, 0), 'size': (1, 3, 10)}
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

# Define the cost function
def cost(current, neighbor):
    return costs[neighbor]

# Define the A* algorithm
def a_star(grid, start, goal):
    open_list = []
    g = {start: 0}
    f = {start: heuristic(start, goal)}
    heapq.heappush(open_list, (f[start], start))
    came_from = {}
    closed_set = set()

    while open_list:
        _, current = heapq.heappop(open_list)
        closed_set.add(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        neighbors = [(0,1,0),(1,0,0),(0,-1,0),(-1,0,0),(0,0,1),(0,0,-1)]
        for dx, dy, dz in neighbors:
            x, y, z = current[0] + dx, current[1] + dy, current[2] + dz
            if 0 <= x < grid_size and 0 <= y < grid_size and 0 <= z < grid_size:
                neighbor = (x, y, z)
                if grid[neighbor] == OBSTACLE or neighbor in closed_set:
                    continue
                tentative_g = g[current] + cost(current, neighbor)
                if neighbor not in g or tentative_g < g[neighbor]:
                    came_from[neighbor] = current
                    g[neighbor] = tentative_g
                    f[neighbor] = g[neighbor] + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f[neighbor], neighbor))

    return []

def reduce_path(path, max_points=5):
    if len(path) <= max_points:
        return path
    step = len(path) // (max_points - 1)
    reduced_path = [path[i] for i in range(0, len(path), step)]
    if len(reduced_path) < max_points:
        reduced_path.append(path[-1])
    return reduced_path


path = a_star(grid, start, goal)
reduced_path = reduce_path(path)
r_path = []
xs, ys, zs = zip(*reduced_path)
for i in range(len(xs)):
    #print(float(xs[i]),",",float(ys[i]),",",float(zs[i]))
    r_path.append((xs[i],ys[i],zs[i]))

# ---------------------------------------------------------
#"""

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
# for _ in range(2):
#     delay_loop.poses.append(make_pose(x=20.0, y=-13.0, z=-7.0))
#     delay_loop.poses.append(make_pose(x=10.0, y=-23.0, z=-7.0))
#     delay_loop.poses.append(make_pose(x=-10.0, y=-8.0, z=-7.0))
#     delay_loop.poses.append(make_pose(x=0.0, y=0.0, z=-7.0))



for i in range(len(xs)):
    delay_loop.poses.append(make_pose(float(r_path[i][0]), float(r_path[i][1]), float(-r_path[i][2])))
    print(float(r_path[i][0]),",",float(r_path[i][1]),",",float(-r_path[i][2]))


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
#"""