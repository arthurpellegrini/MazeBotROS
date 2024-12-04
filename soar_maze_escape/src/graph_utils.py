import numpy as np

# Nodes have these properties:
#     Position
class Node:
    def __init__(self, position):
        self.position = position

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

# Find neighbors in free space
def find_neighbors(grid, x, y):
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and grid[nx, ny] == 0:
            neighbors.append((nx, ny))
    return neighbors

# Perform BFS/DFS to find edges between nodes
def find_edges(grid, start, nodes):
    visited = set()
    stack = [(start, 0, [])]
    edges = []

    while stack:
        (cx, cy), cost, path = stack.pop()
        visited.add((cx, cy))
        path = path + [(cx, cy)]

        if (cx, cy) != start and (cx, cy) in nodes:
            edges.append((start, (cx, cy), cost))
            continue

        for nx, ny in find_neighbors(grid, cx, cy):
            if (nx, ny) not in visited:
                stack.append(((nx, ny), cost + 1, path))

    return edges

# Transform map to graph
def build_graph(grid):
    nodes = {}
    edges = []
    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            if grid[y, x] == 0:  # Assuming 0 represents a free point
                node = Node((x, y))
                nodes[(x, y)] = node
                # Add edges (this is just an example, you need to implement the actual logic)
                if (x-1, y) in nodes:
                    edges.append(((x-1, y), (x, y)))
                if (x, y-1) in nodes:
                    edges.append(((x, y-1), (x, y)))
    return nodes, edges