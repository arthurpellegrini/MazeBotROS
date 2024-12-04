#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import heapq


def get_map() -> OccupancyGrid:
    """Loads map from ROS service."""
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
    """Gets the robot's current position in the map frame."""
    try:
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(1)  # Allow some time for the listener to initialize

        # Get the transform from the robot's base frame to the map frame
        transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        return transform.transform.translation.x, transform.transform.translation.y
    except tf2_ros.LookupException as e:
        rospy.logerr(f"Transform error: {e}")
        raise


def world_to_grid(world_x, world_y, resolution, origin):
    """Converts world coordinates to grid coordinates."""
    grid_x = int((world_x - origin.x) / resolution)
    grid_y = int((world_y - origin.y) / resolution)
    return grid_x, grid_y


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



if __name__ == "__main__":
    rospy.init_node('maze_escape_planner')

    try:
        # Step 1: Retrieve and process the map
        occupancy_grid = get_map()
        grid, resolution, origin = transform_map(occupancy_grid)

        # Step 2: Create graph representation
        graph = create_graph(grid)

        # Step 3: Get robot position and find exits
        robot_x, robot_y = get_robot_position()
        start = world_to_grid(robot_x, robot_y, resolution, origin)
        exits = find_exits(grid)

        # Step 4: Find the shortest path to the closest exit
        shortest_path = None
        for exit in exits:
            path = a_star_search(graph, start, exit)
            if path and (shortest_path is None or len(path) < len(shortest_path)):
                shortest_path = path

        # Step 5: Plot the results
        if shortest_path:
            rospy.loginfo("Path successfully found!")
            plot_map_with_path(grid, shortest_path, resolution, origin)
        else:
            rospy.logwarn("No path to any exit found.")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
