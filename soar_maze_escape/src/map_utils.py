import rospy
import numpy as np

from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid


# Helper method for retrieving the map
def getMap() -> OccupancyGrid:
    """ Loads map from map service """
    # Create service proxy
    get_map = rospy.ServiceProxy('static_map', GetMap)
    # Call service
    recMap = get_map()
    recMap = recMap.map
    # Return
    return recMap

def transformMap(occupancy_grid: OccupancyGrid):
    """ Transforms the OccupancyGrid into Cartesian coordinates for free and wall points. """
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
        return indices * resolution + np.array([origin.y, origin.x])
        # return np.fliplr(coords)  # Flip to (x, y) order

    free_positions = to_cartesian(free_indices)
    wall_positions = to_cartesian(wall_indices)

    return free_positions, wall_positions
