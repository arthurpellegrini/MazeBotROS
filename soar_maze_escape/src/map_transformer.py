#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

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

    # Transform indices into Cartesian coordinates
    def to_world_coords(index):
        x = origin.x + index[1] * resolution
        y = origin.y + index[0] * resolution
        return Point(x=x, y=y, z=0)

    free_positions = [to_world_coords(idx) for idx in free_indices]
    wall_positions = [to_world_coords(idx) for idx in wall_indices]

    return free_positions, wall_positions

def plot_map(free_positions, wall_positions):
    """Plots the transformed map using matplotlib."""
    free_x = [pos.x for pos in free_positions]
    free_y = [pos.y for pos in free_positions]
    wall_x = [pos.x for pos in wall_positions]
    wall_y = [pos.y for pos in wall_positions]

    plt.figure(figsize=(10, 10))
    plt.scatter(free_x, free_y, c='green', s=1, label='Free Space')
    plt.scatter(wall_x, wall_y, c='red', s=1, label='Walls')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Transformed Map')
    plt.show()

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('map_transformer')

    # Retrieve and transform map
    try:
        rospy.wait_for_service('static_map', timeout=10)
        occupancy_grid = get_map()
        free_positions, wall_positions = transform_map(occupancy_grid)
        plot_map(free_positions, wall_positions)
    except rospy.ROSException as e:
        rospy.logerr(f"Service call failed: {e}")
