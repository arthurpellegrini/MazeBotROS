#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import heapq

def get_map() -> OccupancyGrid:
    """Loads map from ROS service."""
    # Create service proxy
    get_map_service = rospy.ServiceProxy('static_map', GetMap)
    # Call service
    received_map = get_map_service()
    return received_map.map

def transform_map(occupancy_grid: OccupancyGrid):
    """Transforms the OccupancyGrid into Cartesian coordinates for free and wall points."""
    # Extract data and metadata
    data = occupancy_grid.data
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    resolution = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin.position

    # Reshape the 1D array into 2D
    grid = np.array(data).reshape((height, width))

    # Find indices of free and wall cells
    free_indices = np.argwhere(grid == 0)  # Free space
    wall_indices = np.argwhere(grid == 100)  # Walls

    return grid, resolution, origin

def create_graph(map_data, resolution, origin):
    """Creates a graph representation from the map."""
    height, width = map_data.shape
    nodes = {}
    for y in range(height):
        for x in range(width):
            if map_data[y, x] == 0:  # Free space
                nodes[(x, y)] = []
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Neighbors (4-connectivity)
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height and map_data[ny, nx] == 0:
                        cost = resolution  # Uniform cost for now
                        nodes[(x, y)].append(((nx, ny), cost))
    return nodes

def a_star_search(graph, start, goal):
    """A* search algorithm to find the shortest path."""
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
    """Heuristic function for A* (Euclidean distance)."""
    return np.linalg.norm(np.array(a) - np.array(b))

def reconstruct_path(came_from, current):
    """Reconstructs the path from the A* algorithm."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def plot_map_with_path(map_data, path, resolution, origin):
    """Plots the map and the global path."""
    plt.figure(figsize=(10, 10))
    walls = np.argwhere(map_data == 100)
    free_space = np.argwhere(map_data == 0)
    plt.scatter(walls[:, 1] * resolution + origin.x, walls[:, 0] * resolution + origin.y, 
                c='red', s=1, label='Walls')
    plt.scatter(free_space[:, 1] * resolution + origin.x, free_space[:, 0] * resolution + origin.y, 
                c='green', s=1, label='Free Space')
    path_x = [p[0] * resolution + origin.x for p in path]
    path_y = [p[1] * resolution + origin.y for p in path]
    plt.plot(path_x, path_y, c='blue', linewidth=2, label='Path')
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.title("Global Path Planning")
    plt.show()

if __name__ == "__main__":
    rospy.init_node('map_transform_and_path_plan')

    # Retrieve and transform map
    try:
        rospy.wait_for_service('static_map', timeout=10)
        occupancy_grid = get_map()
        grid, resolution, origin = transform_map(occupancy_grid)

        # Create graph and find path
        graph = create_graph(grid, resolution, origin)
        start = (0, 0)  # Replace with the actual start point
        goal = (grid.shape[1] - 1, grid.shape[0] - 1)  # Replace with the actual goal point
        path = a_star_search(graph, start, goal)

        if path:
            rospy.loginfo("Path found!")
            plot_map_with_path(grid, path, resolution, origin)
        else:
            rospy.logwarn("Path not found.")
    except rospy.ROSException as e:
        rospy.logerr(f"Service call failed: {e}")
