#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import heapq

# Global publisher for velocity commands
cmd_vel_pub = None


def get_map() -> OccupancyGrid:
    """Loads the map from the ROS service."""
    try:
        rospy.wait_for_service('static_map', timeout=10)
        get_map_service = rospy.ServiceProxy('static_map', GetMap)
        received_map = get_map_service()
        return received_map.map
    except rospy.ROSException as e:
        rospy.logerr(f"Service call failed: {e}")
        raise


def transform_map(occupancy_grid: OccupancyGrid):
    """Transforms the OccupancyGrid into a 2D numpy array."""
    data = occupancy_grid.data
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    resolution = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin.position

    grid = np.array(data).reshape((height, width))
    return grid, resolution, origin


def create_graph(grid):
    """Creates a graph representation of the map."""
    graph = {}
    height, width = grid.shape

    for y in range(height):
        for x in range(width):
            if grid[y, x] == 0:  # Free space
                graph[(x, y)] = []
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4-connectivity
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height and grid[ny, nx] == 0:
                        graph[(x, y)].append(((nx, ny), 1))  # Uniform cost
    return graph


def get_robot_position():
    """Gets the robot's current position and orientation in the map frame."""
    try:
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(1)  # Allow some time for the listener to initialize

        transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        x = transform.transform.translation.x
        y = transform.transform.translation.y

        # Extract the robot's orientation as a quaternion and convert to yaw (theta)
        q = transform.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny_cosp, cosy_cosp)

        return x, y, theta
    except tf2_ros.LookupException as e:
        rospy.logerr(f"Transform error: {e}")
        raise


def world_to_grid(world_x, world_y, resolution, origin):
    """Converts world coordinates to grid coordinates."""
    grid_x = int(round((world_x - origin.x) / resolution))
    grid_y = int(round((world_y - origin.y) / resolution))
    return max(0, grid_x), max(0, grid_y)  # Ensure within bounds


def grid_to_world(grid_x, grid_y, resolution, origin):
    """Converts grid coordinates to world coordinates."""
    world_x = grid_x * resolution + origin.x
    world_y = grid_y * resolution + origin.y
    return world_x, world_y


def find_exits(grid):
    """Finds the exit nodes (free cells on the boundary)."""
    height, width = grid.shape
    exits = []

    for x in range(width):
        if grid[0, x] == 0:
            exits.append((x, 0))  # Top boundary
        if grid[height - 1, x] == 0:
            exits.append((x, height - 1))  # Bottom boundary

    for y in range(height):
        if grid[y, 0] == 0:
            exits.append((0, y))  # Left boundary
        if grid[y, width - 1] == 0:
            exits.append((width - 1, y))  # Right boundary

    return exits


def a_star_search(graph, start, goal):
    """A* search algorithm."""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic(start, goal)

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor, cost in graph[current]:
            tentative_g_score = g_score[current] + cost
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None  # Path not found


def heuristic(a, b):
    """Heuristic function: Manhattan distance."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct_path(came_from, current):
    """Reconstructs the path from A* results."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def smooth_path(path):
    """Applies basic smoothing to reduce sharp turns."""
    smoothed_path = [path[0]]
    for i in range(1, len(path) - 1):
        prev = np.array(path[i - 1])
        curr = np.array(path[i])
        next = np.array(path[i + 1])
        avg = (prev + next) / 2.0
        smoothed_path.append(tuple(avg.astype(int)))
    smoothed_path.append(path[-1])
    return smoothed_path


def calculate_velocity_to_target(robot_x, robot_y, waypoint_x, waypoint_y):
    """Calculates linear and angular velocity to reach the target waypoint."""
    kp_linear = 0.5
    kp_angular = 2.0
    dx = waypoint_x - robot_x
    dy = waypoint_y - robot_y
    distance = np.sqrt(dx**2 + dy**2)
    angle_to_target = np.arctan2(dy, dx)

    # Normalize angle to [-pi, pi]
    angular_diff = np.arctan2(np.sin(angle_to_target), np.cos(angle_to_target))

    # Stop moving if close enough
    if distance < 0.1:
        return 0.0, 0.0

    angular = kp_angular * angular_diff
    linear = min(kp_linear * distance, 0.5)  # Cap max speed
    return linear, angular


def publish_velocity(linear, angular):
    """Publishes velocity commands to the robot."""
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    cmd_vel_pub.publish(twist)


def align_robot(robot_x, robot_y, robot_theta, waypoint_x, waypoint_y):
    """Align the robot to face the first waypoint."""
    kp_angular = 2.0
    angle_to_target = np.arctan2(waypoint_y - robot_y, waypoint_x - robot_x)

    # Calculate angular difference relative to the robot's current heading
    angular_diff = angle_to_target - robot_theta
    angular_diff = np.arctan2(np.sin(angular_diff), np.cos(angular_diff))  # Normalize to [-pi, pi]

    rospy.loginfo(f"Aligning robot. Current theta: {robot_theta:.2f}, Target angle: {angle_to_target:.2f}")

    while abs(angular_diff) > 0.05:  # Threshold for alignment (radians)
        angular_velocity = kp_angular * angular_diff
        publish_velocity(0.0, angular_velocity)  # Only rotate
        rospy.sleep(0.1)

        # Update robot's orientation dynamically
        _, _, robot_theta = get_robot_position()
        angular_diff = angle_to_target - robot_theta
        angular_diff = np.arctan2(np.sin(angular_diff), np.cos(angular_diff))  # Normalize
    publish_velocity(0.0, 0.0)  # Stop rotation
    rospy.loginfo("Robot aligned successfully.")


def main():
    rospy.init_node('maze_escape_planner')
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    try:
        occupancy_grid = get_map()
        grid, resolution, origin = transform_map(occupancy_grid)
        graph = create_graph(grid)

        robot_x, robot_y, robot_theta = get_robot_position()
        start = world_to_grid(robot_x, robot_y, resolution, origin)

        if grid[start[1], start[0]] != 0:
            rospy.logerr("Robot is starting in a wall!")
            return

        exits = find_exits(grid)
        shortest_path = None
        for exit in exits:
            path = a_star_search(graph, start, exit)
            if path and (shortest_path is None or len(path) < len(shortest_path)):
                shortest_path = path

        if shortest_path:
            rospy.loginfo("Path successfully found!")
            shortest_path = smooth_path(shortest_path)

            for waypoint in shortest_path:
                robot_x, robot_y, robot_theta = get_robot_position()
                waypoint_x, waypoint_y = grid_to_world(waypoint[0], waypoint[1], resolution, origin)
                align_robot(robot_x, robot_y, robot_theta, waypoint_x, waypoint_y)
                linear, angular = calculate_velocity_to_target(robot_x, robot_y, waypoint_x, waypoint_y)
                publish_velocity(linear, angular)
                rospy.sleep(0.1)
            publish_velocity(0, 0)
            rospy.loginfo("Robot has reached the exit!")
        else:
            rospy.logwarn("No valid path to the exit found.")
    except Exception as e:
        rospy.logerr(f"Error: {e}")


if __name__ == "__main__":
    main()
