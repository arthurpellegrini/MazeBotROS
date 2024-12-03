#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
import heapq
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


# Global publisher for velocity commands
cmd_vel_pub = None

def get_map() -> OccupancyGrid:
    """Loads the map from the ROS service."""
    rospy.wait_for_service('static_map', timeout=10)
    get_map_service = rospy.ServiceProxy('static_map', GetMap)
    received_map = get_map_service()
    return received_map.map


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
    """Gets the robot's current position in the map frame."""
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1)

    transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
    x = transform.transform.translation.x
    y = transform.transform.translation.y

    # Extract the robot's orientation as a quaternion and convert to yaw (theta)
    q = transform.transform.rotation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    theta = np.arctan2(siny_cosp, cosy_cosp)

    return x, y, theta


def world_to_grid(world_x, world_y, resolution, origin):
    """Converts world coordinates to grid coordinates."""
    grid_x = int((world_x - origin.x) / resolution)
    grid_y = int((world_y - origin.y) / resolution)
    return grid_x, grid_y


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
    return None

def publish_path_marker(path, resolution, origin):
    """Publishes the A* path as a line marker in Rviz."""
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "a_star_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.05  # Line width in meters

    # Line color (RGBA)
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    # Convert path grid points to world coordinates
    marker.points = []
    for grid_x, grid_y in path:
        world_x, world_y = grid_to_world(grid_x, grid_y, resolution, origin)
        marker.points.append(Point(world_x, world_y, 0.0))  # z=0 for 2D visualization

    # Publish the marker
    path_marker_pub.publish(marker)
    rospy.loginfo("Path marker published to Rviz.")


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


def follow_path(path, resolution, origin):
    """Follows the path generated by A*."""
    for grid_point in path:
        waypoint_x, waypoint_y = grid_to_world(grid_point[0], grid_point[1], resolution, origin)
        align_and_move(waypoint_x, waypoint_y)


def align_and_move(waypoint_x, waypoint_y):
    """Aligns the robot to the waypoint and moves towards it."""
    robot_x, robot_y, robot_theta = get_robot_position()
    dx = waypoint_x - robot_x
    dy = waypoint_y - robot_y
    distance = np.sqrt(dx**2 + dy**2)

    # Align robot
    target_angle = np.arctan2(dy, dx)
    angular_diff = target_angle - robot_theta
    angular_diff = np.arctan2(np.sin(angular_diff), np.cos(angular_diff))  # Normalize to [-pi, pi]

    # Aligning loop
    while abs(angular_diff) > 0.1:  # Threshold for alignment (radians)
        angular_velocity = max(min(angular_diff * 2.0, 1.0), -1.0)  # Limit angular speed
        publish_velocity(0, angular_velocity)  # Rotate towards target
        rospy.sleep(0.1)

        # Update robot's orientation dynamically
        _, _, robot_theta = get_robot_position()
        angular_diff = target_angle - robot_theta
        angular_diff = np.arctan2(np.sin(angular_diff), np.cos(angular_diff))  # Normalize again

    publish_velocity(0, 0)  # Stop rotation

    # Move robot
    while distance > 0.1:  # Threshold for reaching the waypoint
        robot_x, robot_y, _ = get_robot_position()
        dx = waypoint_x - robot_x
        dy = waypoint_y - robot_y
        distance = np.sqrt(dx**2 + dy**2)
        linear_velocity = min(distance * 0.5, 0.3)  # Cap max speed
        publish_velocity(linear_velocity, 0)  # Move forward
        rospy.sleep(0.1)

    publish_velocity(0, 0)  # Stop robot



def publish_velocity(linear, angular):
    """Publishes velocity commands to the robot."""
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    cmd_vel_pub.publish(twist)

def plot_map_with_path(grid, path, resolution, origin):
    """Plots the map with improved visualization for walls, free space, and the computed path."""
    plt.figure(figsize=(12, 12))

    # Extract walls, free space, and path data
    walls = np.argwhere(grid == 100)
    free_space = np.argwhere(grid == 0)

    # Plot walls (red squares)
    plt.scatter(
        walls[:, 1] * resolution + origin.x,
        walls[:, 0] * resolution + origin.y,
        c='black', s=10, label='Walls', marker='s'
    )

    # Plot free space (light gray dots)
    plt.scatter(
        free_space[:, 1] * resolution + origin.x,
        free_space[:, 0] * resolution + origin.y,
        c='lightgray', s=1, label='Free Space'
    )

    # Plot the path (blue line with larger points for emphasis)
    if path:
        path_x = [p[0] * resolution + origin.x for p in path]
        path_y = [p[1] * resolution + origin.y for p in path]
        plt.plot(path_x, path_y, c='blue', linewidth=2, label='Path')  # Blue line for the path
        plt.scatter(path_x, path_y, c='blue', s=15, label='Path Points')  # Larger dots for path points

    # Highlight start and goal points with distinct markers
    if path:
        plt.scatter(
            path_x[0], path_y[0], c='green', s=100, label='Start', marker='o'
        )  # Start (green circle)
        plt.scatter(
            path_x[-1], path_y[-1], c='red', s=100, label='Exit', marker='x'
        )  # Exit (red cross)

    # Add labels, title, and legend
    plt.xlabel("X-Coordinate (m)")
    plt.ylabel("Y-Coordinate (m)")
    plt.title("Maze Map with Global Path", fontsize=16)
    plt.legend(loc='upper right', fontsize=12)
    plt.grid(color='gray', linestyle='--', linewidth=0.5)

    # Show the plot
    plt.tight_layout()
    plt.show()

def main():
    rospy.init_node('local_path_follower')
    global path_marker_pub, cmd_vel_pub

    # Initialize publishers
    path_marker_pub = rospy.Publisher('/a_star_path', Marker, queue_size=10)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Allow time for publishers to initialize
    rospy.logerr("You can add the marker in Rviz, be quick")
    rospy.sleep(5)
    if not path_marker_pub:
        rospy.logerr("Path marker publisher is not initialized.")
        return
    if not cmd_vel_pub:
        rospy.logerr("Velocity command publisher is not initialized.")
        return

    try:
        occupancy_grid = get_map()
        grid, resolution, origin = transform_map(occupancy_grid)
        graph = create_graph(grid)

        robot_x, robot_y, robot_theta = get_robot_position()
        start = world_to_grid(robot_x, robot_y, resolution, origin)

        exits = find_exits(grid)
        rospy.loginfo(f"Exits found: {exits}")

        paths = [a_star_search(graph, start, e) for e in exits]
        valid_paths = [path for path in paths if path]
        if not valid_paths:
            rospy.logwarn("No valid paths found to any exit.")
            return
        shortest_path = min(valid_paths, key=len)

        rospy.loginfo(f"Shortest path: {shortest_path}")
        
        # Plot the map and the planned path
        plot_map_with_path(grid, shortest_path, resolution, origin)

        # Publish the path marker for Rviz visualization
        publish_path_marker(shortest_path, resolution, origin)

        # Follow the path
        follow_path(shortest_path, resolution, origin)
        rospy.loginfo("Robot has exited the maze.")
    except Exception as e:
        rospy.logerr(f"Error: {e}")




if __name__ == "__main__":
    main()
