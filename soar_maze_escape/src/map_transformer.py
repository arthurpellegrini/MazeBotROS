#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import os

def get_map() -> OccupancyGrid:
    """
    Loads the map from the ROS service.
    """
    rospy.wait_for_service('static_map', timeout=10)
    try:
        get_map_service = rospy.ServiceProxy('static_map', GetMap)
        response = get_map_service()
        return response.map
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to 'static_map' failed: {e}")
        raise

def transform_map(occupancy_grid: OccupancyGrid):
    """
    Transforms the OccupancyGrid into Cartesian coordinates for free and wall points.

    Args:
        occupancy_grid (OccupancyGrid): The map data from ROS.

    Returns:
        tuple: (free_positions, wall_positions) as NumPy arrays.
    """
    # Extract map data and metadata
    resolution = occupancy_grid.info.resolution
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    origin = occupancy_grid.info.origin.position

    # Convert the 1D array into a 2D grid
    grid = np.array(occupancy_grid.data).reshape((height, width))

    # Find indices for free and wall cells
    free_indices = np.argwhere(grid == 0)  # Free space
    wall_indices = np.argwhere(grid == 100)  # Walls

    # Transform indices to Cartesian coordinates
    def to_cartesian(indices):
        coords = indices * resolution + np.array([origin.y, origin.x])
        return np.fliplr(coords)  # Flip to (x, y) order

    free_positions = to_cartesian(free_indices)
    wall_positions = to_cartesian(wall_indices)

    return free_positions, wall_positions

def plot_map(free_positions, wall_positions, save_to_file=True):
    """
    Plots the transformed map with improved visuals for clarity.

    Args:
        free_positions (ndarray): Cartesian coordinates of free positions.
        wall_positions (ndarray): Cartesian coordinates of wall positions.
        save_to_file (bool): Whether to save the plot as a file.
    """
    plt.figure(figsize=(7, 7))
    
    # Scatter plot for walls
    plt.scatter(wall_positions[:, 0], wall_positions[:, 1], 
                c="darkblue", label="Walls", s=50, alpha=0.9, marker="s")
    
    # Scatter plot for free space
    plt.scatter(free_positions[:, 0], free_positions[:, 1], 
                c="lightgray", label="Unobstructed Space", s=20, alpha=0.3)

    # Styling
    plt.title("Map Data Transformed into World Coordinates", fontsize=14)
    plt.xlabel("X-Coordinate [m]", fontsize=12)
    plt.ylabel("Y-Coordinate [m]", fontsize=12)
    plt.legend()
    plt.grid(color="lightgray", linestyle="--", linewidth=0.5)
    plt.tight_layout()

    if save_to_file:
        output_dir = "results"
        os.makedirs(output_dir, exist_ok=True)
        file_path = os.path.join(output_dir, 'transformed_map.png')
        plt.savefig(file_path)
        rospy.loginfo(f"Map visualization saved to {file_path}")

    plt.show()


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('map_transformer', anonymous=True)

    # Retrieve and transform the map
    try:
        rospy.loginfo("Waiting for the static_map service...")
        occupancy_grid = get_map()
        rospy.loginfo("Map retrieved successfully!")

        rospy.loginfo("Transforming the map...")
        free_positions, wall_positions = transform_map(occupancy_grid)

        rospy.loginfo("Visualizing the map...")
        plot_map(free_positions, wall_positions)

        rospy.loginfo("Map transformation and visualization completed.")
    except Exception as e:
        rospy.logerr(f"Error occurred: {e}")
