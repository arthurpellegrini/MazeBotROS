import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import numpy as np

def publish_path_marker(path, resolution, origin, path_marker_pub):
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
        world_x = grid_x * resolution + origin.x
        world_y = grid_y * resolution + origin.y
        marker.points.append(Point(world_x, world_y, 0.0))  # z=0 for 2D visualization

    path_marker_pub.publish(marker)
    rospy.loginfo("Path marker published to Rviz.")

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
