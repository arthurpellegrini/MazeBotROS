#!/usr/bin/env python3

import rospy
from map_utils import getMap, transformMap
from graph_utils import buildGraph, addExits
from visualization_utils import plotMap, plotGraph
from robot_controller import localiseRobot, goToNode
import numpy as np

def main():
    rospy.init_node("moro_maze_navigation")
    recMap = getMap()
    freepoints, wallpoints = transformMap(recMap)
    # plotMap(freepoints, wallpoints)

    # Convert map to 2D grid
    grid = np.array(recMap.data).reshape((recMap.info.height, recMap.info.width))

    # Build graph
    nodes, edges = buildGraph(grid)
    for edge in edges:
        print(edge)

    # Add exits to the graph
    nodes, edges = addExits(grid, nodes, edges)

    robot_pos = localiseRobot()
    print("robot pose",robot_pos)
    plotGraph(recMap, edges, wallpoints, robot_pos)

    # exemple node ([y, x])
    goToNode(robot_pos, edges[0].parent, recMap)

if __name__ == "__main__":
    main()
