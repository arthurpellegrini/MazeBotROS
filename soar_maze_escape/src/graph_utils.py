PADDING = 4
SIZE = 6

# Nodes have these properties:
#     Position
class Node:
    def __init__(self, position):
        self.position = position

    def get_position(self):
        return self.position

    def __str__(self):
        return str(self.position[0]) + "." + str(self.position[1])

# Edges have these properties:
#     Parent Node
#     Child Node
#     Cost
class Edge:
    def __init__(self, parent, child, cost):
        self.parent = parent
        self.child = child
        self.cost = cost
    
    def get_cost(self):
        return self.cost
    
    def get_parent(self):
        return self.parent
    
    def get_child(self):
        return self.child
    
    def __str__(self):
        return "{'parent': '" + str(self.parent) + "'" + \
               ",'child': '" + str(self.child) + "'}"

# Check if a position is a node (intersection, dead end, etc.)
def is_node(grid, x, y):
    if grid[x, y] != 0:  # Not free space
        return False

    # Count free neighbors
    neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
    free_neighbors = sum(0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and grid[nx, ny] == 0 for nx, ny in neighbors)

    # A node has more than 2 free neighbors or is a dead end
    return free_neighbors != 2

def has_wall(grid, start, end):
    """
    Checks if there is a wall between the start and end points.
    
    Args:
        grid (numpy.ndarray): The grid map.
        start (tuple): Starting coordinates (x1, y1).
        end (tuple): Ending coordinates (x2, y2).
    
    Returns:
        bool: True if a wall is found, False otherwise.
    """
    x1, y1 = start
    x2, y2 = end

    # Determine the number of steps needed to traverse between start and end
    dx = x2 - x1
    dy = y2 - y1
    steps = max(abs(dx), abs(dy))
    
    # Check points along the path, excluding start and end
    for step in range(1, steps):
        x = x1 + step * dx // steps
        y = y1 + step * dy // steps

        # Check if the current position is a wall
        if grid[y, x] != 0:  # Assuming non-zero values indicate walls
            return True

    return False


def find_neighbors(grid, x, y, far_from=SIZE):
    neighbors = []
    directions = [(-far_from, 0), (far_from, 0), (0, -far_from), (0, far_from)]
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
            if grid[ny, nx] == 0:
                if not has_wall(grid, (x, y), (nx, ny)):
                    neighbors.append((nx, ny))
                else:
                    continue
    return neighbors

def build_graph(grid):
    """
    Constructs a graph from the grid, identifying nodes and edges.
    
    Args:
        grid (numpy.ndarray): The occupancy grid map.
    
    Returns:
        tuple: A dictionary of nodes and a list of edges.
    """
    nodes = {}
    edges = []

    # Iterate through the grid with steps based on SIZE
    for y in range(PADDING, grid.shape[0] - 1, SIZE):
        for x in range(PADDING, grid.shape[1] - 1, SIZE):
            if grid[y, x] == 0:  # Check if the current cell is a valid node
                node = Node((x, y))
                nodes[(x, y)] = node

                # Find neighbors and create edges
                for nx, ny in find_neighbors(grid, x, y):
                    if (nx, ny) in nodes:
                        if not has_wall(grid, (x, y), (nx, ny)):
                            edges.append(Edge(nodes[(x, y)], nodes[(nx, ny)], cost=SIZE))

    return nodes, edges


def add_exits(grid, nodes, edges):
    """
    Adds exit nodes to the graph by checking boundary nodes without walls towards the outside.
    
    Args:
        grid (numpy.ndarray): The occupancy grid map.
        nodes (dict): Existing nodes in the graph.
        edges (list): Existing edges in the graph.
    
    Returns:
        tuple: Updated nodes and edges with added exits.
    """
    height, width = grid.shape
    # exits = []

    def is_an_exit_neighbor(grid, nodes, x, y):
        # Find neighbors and create edges
        for nx, ny in find_neighbors(grid, x, y, 4):
            if (nx, ny) in nodes:
                if not has_wall(grid, (x, y), (nx, ny)):
                    node_exit = Node((x, y))
                    return Edge(node_exit, nodes[(nx, ny)], cost=SIZE)
        return None

    for x in range(width):
        if grid[0, x] == 0:
            exit = is_an_exit_neighbor(grid, nodes, x, 0)
            if exit:
                edges.append(exit)
        if grid[height - 1, x] == 0:
            exit = is_an_exit_neighbor(grid, nodes, x, height - 1)
            if exit:
                edges.append(exit)

    for y in range(height):
        if grid[y, 0] == 0:
            exit = is_an_exit_neighbor(grid, nodes, 0, y)
            if exit:
                edges.append(exit)
        if grid[y, width - 1] == 0:
            exit = is_an_exit_neighbor(grid, nodes, width - 1, y)
            if exit:
                edges.append(exit)


    return nodes, edges

