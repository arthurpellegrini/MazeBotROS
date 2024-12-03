import rospy
from nav_msgs.srv import GetMap
import numpy as np

def get_map():
    rospy.wait_for_service('static_map', timeout=10)
    get_map_service = rospy.ServiceProxy('static_map', GetMap)
    return get_map_service().map

def transform_map(occupancy_grid):
    data = occupancy_grid.data
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    resolution = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin.position

    grid = np.array(data).reshape((height, width))
    return grid, resolution, origin

def find_exits(grid):
    height, width = grid.shape
    exits = []

    for x in range(width):
        if grid[0, x] == 0:
            exits.append((x, 0))
        if grid[height - 1, x] == 0:
            exits.append((x, height - 1))

    for y in range(height):
        if grid[y, 0] == 0:
            exits.append((0, y))
        if grid[y, width - 1] == 0:
            exits.append((width - 1, y))

    return exits
