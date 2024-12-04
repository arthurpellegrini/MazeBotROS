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

# # Find neighbors in free space
# def find_neighbors(grid, x, y):
#     neighbors = []
#     for dx, dy in [(-5, 0), (5, 0), (0, -5), (0, 5)]:
#         nx, ny = x + dx, y + dy
#         if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0] and grid[ny, nx] == 0:
#             neighbors.append((nx, ny))
#     return neighbors

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

def has_wall(grid, start, end):
    x1, y1 = start
    x2, y2 = end
    if x1 == x2:
        step = 6 if y2 > y1 else -6
        for y in range(y1, y2, step):
            if grid[y, x1] == 1:
                return True
    elif y1 == y2:
        step = 6 if x2 > x1 else -6
        for x in range(x1, x2, step):
            if grid[y1, x] == 1:
                return True
    return False

def find_neighbors(grid, x, y):
    neighbors = []
    directions = [(-6, 0), (6, 0), (0, -6), (0, 6)]
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
            if grid[ny, nx] == 0 and not has_wall(grid, (x, y), (nx, ny)):
                neighbors.append((nx, ny))
    return neighbors

def build_graph(grid):
    nodes = {}
    edges = []
    for y in range(4, grid.shape[0]-1, 6):
        for x in range(4, grid.shape[1]-1, 6):
            if grid[y, x] == 0:  # Pas de mur sur le nœud
                node = Node((x, y))
                nodes[(x, y)] = node
                for nx, ny in find_neighbors(grid, x, y):
                    if (nx, ny) in nodes:
                        edges.append(((x, y), (nx, ny)))
    return nodes, edges