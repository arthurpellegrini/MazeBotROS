PADDING = 4
SIZE = 6

import numpy as np
from map_utils import convertWorldToGrid
 
# Node class represents a graph node with a position
class Node:
    def __init__(self, position):
        self.position = position

    def __str__(self):
        return f"{self.position[0]}.{self.position[1]}"

# Edge class represents a graph edge with parent, child, and cost
class Edge:
    def __init__(self, parent, child, cost):
        self.parent = parent
        self.child = child
        self.cost = cost

    def __str__(self):
        return f"{{'parent': '{self.parent}', 'child': '{self.child}'}}"

# Determines if a grid cell is a node (intersection, dead end, etc.)
def isNode(grid: np.array, x: int, y: int) -> bool:
    """
    Determines if a grid cell is a node (intersection, dead end, etc.)
    
    Args:
        grid (np.array): 2D grid representing the map
        x (int): x-coordinate of the cell
        y (int): y-coordinate of the cell
        
    Returns:
        (bool): True if the cell is a node, False otherwise
    """
    if grid[x, y] != 0:  # Not free space
        return False

    neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
    free_neighbors = sum(
        0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and grid[nx, ny] == 0
        for nx, ny in neighbors
    )

    # A node has more than 2 free neighbors or is a dead end
    return free_neighbors != 2

# Checks if a wall exists between two points
def hasWall(grid: np.array, start, end) -> bool:
    """
    Checks if a wall exists between two points

    Args:
        grid (np.array): 2D grid representing the map
        start (tuple[int, int]): Starting point
        end (tuple[int, int]): Ending point

    Returns:
        (bool): True if a wall exists between the two points, False otherwise
    """
    x1, y1 = start
    x2, y2 = end

    dx, dy = x2 - x1, y2 - y1
    steps = max(abs(dx), abs(dy))

    for step in range(1, steps):
        x = x1 + step * dx // steps
        y = y1 + step * dy // steps

        if grid[y, x] != 0:  # Non-zero indicates a wall
            return True

    return False

# Finds neighbors of a node based on grid constraints and connectivity
def findNeighbors(grid: np.array, x: int, y: int, distance: int=SIZE) -> list:
    """
    Finds neighbors of a node based on grid constraints and connectivity

    Args:
        grid (np.array): 2D grid representing the map
        x (int): x-coordinate of the node
        y (int): y-coordinate of the node
        distance (int): Distance between nodes
    
    Returns:
        (list[tuple[int, int]]): List of neighboring nodes    
    """
    neighbors = []
    directions = [(-distance, 0), (distance, 0), (0, -distance), (0, distance)]
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0] and grid[ny, nx] == 0:
            if not hasWall(grid, (x, y), (nx, ny)):
                neighbors.append((nx, ny))
    return neighbors

# Builds a graph from a 2D grid
def buildGraph(grid: np.array) -> tuple:
    """
    Builds a graph from a 2D grid

    Args:
        grid (np.array): 2D grid representing the map
    
    Returns:
        (list[Edge]): List of edges in the graph
    """
    nodes = {}
    edges = []

    for y in range(PADDING, grid.shape[0] - 1, SIZE):
        for x in range(PADDING, grid.shape[1] - 1, SIZE):
            if grid[y, x] == 0:  # Check if the cell is a valid node
                node = Node((x, y))
                nodes[(x, y)] = node

                for neighbor_x, neighbor_y in findNeighbors(grid, x, y):
                    if (neighbor_x, neighbor_y) in nodes:
                        edges.append(
                            Edge(nodes[(x, y)], nodes[(neighbor_x, neighbor_y)], cost=SIZE)
                        )

    return nodes, edges

# Find exit nodes inside the map
def findExits(grid: np.array, nodes: list) -> list:
    """
    Find exit nodes to the graph by checking boundary nodes without walls towards the outside.

    Args:
        grid (np.array): 2D grid representing the map
        nodes (list[Node]): List of nodes in the graph
    
    Returns:
        (list[Edge]): List of nodes in the graph with exit nodes
    """
        

    height, width = grid.shape
    finded_exits = []
    tmp_nodes = nodes.copy()

    for x in range(width):
        # Check top and bottom boundaries
        for y in [0, height - 1]:
            if grid[y, x] == 0:  # Check if it's an exit point
                for neighbor_x, neighbor_y in findNeighbors(grid, x, y, distance=4):
                    if (neighbor_x, neighbor_y) in nodes:
                        exit_node = Node((x, y))
                        tmp_nodes[(x, y)] = exit_node
                        finded_exits.append(
                            Edge(nodes[(neighbor_x, neighbor_y)], exit_node, cost=SIZE)
                        )

    for y in range(height):
        # Check left and right boundaries
        for x in [0, width - 1]:
            if grid[y, x] == 0:  # Check if it's an exit point
                for neighbor_x, neighbor_y in findNeighbors(grid, x, y, distance=4):
                    if (neighbor_x, neighbor_y) in nodes:
                        exit_node = Node((x, y))
                        tmp_nodes[(x, y)] = exit_node
                        finded_exits.append(
                            Edge(nodes[(neighbor_x, neighbor_y)], exit_node, cost=SIZE)
                        )

    return tmp_nodes, finded_exits

def findClosestNode(robot_pos_world, edges, recMap):
   
    y_robot, x_robot = robot_pos_world[:2]
    y_robot_grid, x_robot_grid = convertWorldToGrid(y_robot, x_robot, recMap)

    # We take edges parent and child nodes to find the closest node
    min_distance = float("inf")
    closest_node = None
    for edge in edges:
        parent = edge.parent
        child = edge.child
        y_parent, x_parent = parent.position
        y_child, x_child = child.position   

        # We calculate the distance from the robot to the parent
        distance_to_parent = np.sqrt((np.square(y_robot_grid - y_parent) + np.square(x_robot_grid - x_parent)))
        if distance_to_parent < min_distance:
            min_distance = distance_to_parent
            closest_node = parent

        # We calculate the distance from the robot to the child
        distance_to_child = np.sqrt((np.square(y_robot_grid - y_child) + np.square(x_robot_grid - x_child)))
        if distance_to_child < min_distance:
            min_distance = distance_to_child
            closest_node = child

    return closest_node