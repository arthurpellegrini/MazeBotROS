PADDING = 4
SIZE = 6

import numpy as np
from map_utils import convertWorldToGrid, convertMapToWorldCoordinates
 
# Node class represents a graph node with a position
class Node:
    def __init__(self, position):
        self.position = position

    def __eq__(self, other):
        return self.position == other.position

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
   
    x_robot, y_robot = robot_pos_world[:2]
    y_robot_grid, x_robot_grid = convertWorldToGrid(y_robot, x_robot, recMap)

    # We take edges parent and child nodes to find the closest node
    min_distance = float("inf")
    closest_node = None
    for edge in edges:
        parent = edge.parent
        child = edge.child
        x_parent, y_parent = parent.position
        x_child, y_child = child.position   

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


import heapq  # For priority queue

def heuristic(a, b):
    """Calculate the heuristic (Euclidean distance) between two points."""
    return np.linalg.norm(np.array(a) - np.array(b))

def a_star_search(grid, nodes, edges, start, goal):
    """
    Perform A* search on the graph to find the shortest path.
    
    Args:
        grid (np.array): 2D grid representing the map.
        nodes (dict): Dictionary of nodes in the graph, keyed by position.
        edges (list[Edge]): List of edges in the graph.
        start (Node): Starting node.
        goal (Node): Goal node.
    
    Returns:
        list[Node]: The shortest path as a list of Node objects.
    """
    open_set = []
    heapq.heappush(open_set, (0, start.position))  # Priority queue with (cost, position)
    
    came_from = {}  # Track the path (position -> position)
    g_score = {node: float('inf') for node in nodes}
    g_score[start.position] = 0
    
    f_score = {node: float('inf') for node in nodes}
    f_score[start.position] = heuristic(start.position, goal.position)
    
    while open_set:
        _, current_position = heapq.heappop(open_set)
        
        if current_position == goal.position:
            # Reconstruct path as a list of Nodes
            path = []
            while current_position in came_from:
                path.append(nodes[current_position])
                current_position = came_from[current_position]
            path.append(start)
            return path[::-1]  # Reverse the path
        
        for edge in edges:
            # Determine the neighbor node based on the current position
            if edge.parent.position == current_position:
                neighbor_position = edge.child.position
            elif edge.child.position == current_position:
                neighbor_position = edge.parent.position
            else:
                continue
            
            tentative_g_score = g_score[current_position] + edge.cost
            if tentative_g_score < g_score[neighbor_position]:
                came_from[neighbor_position] = current_position
                g_score[neighbor_position] = tentative_g_score
                f_score[neighbor_position] = g_score[neighbor_position] + heuristic(neighbor_position, goal.position)
                
                if neighbor_position not in [node for _, node in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor_position], neighbor_position))
    
    return []  # No path found

def interpolate_point(start, end):
    ratio = 0.5  # Midpoint
    x = start[0] + ratio * (end[0] - start[0])
    y = start[1] + ratio * (end[1] - start[1])
    return (x, y)

def reconstruct_path_with_rotation(path, recMap):
    # Add rotation for each step
    global_path = []

    for i in range(0, len(path)):
        if i == 0:
            current_node = path[i]
        else:
            current_node = path[i - 1]
        next_node = path[i]
        # print(f"From {current_node.position} to {next_node.position}")
        
        # Compute the rotation
        dx = next_node.position[0] - current_node.position[0]
        dy = next_node.position[1] - current_node.position[1]
        
        rotation = np.arctan2(dy, dx)  # For other directions
        
        # Interpolate a single point between current_node and next_node
        intermediate_point = interpolate_point(current_node.position, next_node.position)
        
        x_coordinate, y_coordinate = convertMapToWorldCoordinates(intermediate_point[0], intermediate_point[1], recMap)
        global_path.append((x_coordinate, y_coordinate, rotation))
    
    return global_path


def reconstruct_best_path(nodes, start_node, edges, find_exits, grid, recMap) -> list:
    global_path = []

    for pos in nodes:
        if start_node.position == nodes[pos].position:
            start_exits = True

    if start_exits:
        paths = [a_star_search(grid, nodes, edges, start_node, exit.child) for exit in find_exits]
        print("Paths found:", [[step.position for step in path] for path in paths])
        # Get the path with the lower number of steps
        path = min(paths, key=lambda x: len(x)) if paths else None
        print("Path found:", [step.position for step in path])
        global_path = reconstruct_path_with_rotation(path, recMap)
        print("Global path:", global_path)
    else:
        print("Start or goal node is invalid.")
    
    return global_path
