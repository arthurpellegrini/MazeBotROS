#!/usr/bin/env python3

import rospy
from map_utils import getMap, transformMap
from graph_utils import build_graph
from visualization_utils import plotMap, plotGraph
import numpy as np

def main():
    rospy.init_node("moro_maze_navigation")
    recMap = getMap()
    freepoints, wallpoints = transformMap(recMap)
    # plotMap(freepoints, wallpoints)

    # Convert map to 2D grid
    grid = np.array(recMap.data).reshape((recMap.info.height, recMap.info.width))

    # Build graph
    nodes, edges = build_graph(grid)

    # edges = [((4, 4), (10,4)), 
    #         ((4,10), (10,10)),
    #         ((4,16), (4,22)),
    #         ((4,22), (10,22)),
    #         ((10,4), (10,10)),
    #         ((10,10), (10,16)),
    #         ((10,16), (16,16)),
    #         ((10,16), (10,22)),
    #         ((16,4), (16,10)),
    #         ((16,10), (22,10))]

    # Print nodes and edges
    print("Nodes:")
    for node in nodes.values():
        print(node)

    print("\nEdges:")
    for edge in edges:
        print(edge)

    robot_pos = (1, 2) # (y, x)
    plotGraph(recMap, edges, wallpoints, robot_pos)

if __name__ == "__main__":
    main()
