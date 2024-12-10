#!/usr/bin/env python3

import rospy
from map_utils import getMap, transformMap
from graph_utils import buildGraph, findExits
from visualization_utils import plotMap, plotGraph, plotNodePositionGraph
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

    # Add exits to the graph
    nodes, find_exits = findExits(grid, nodes)
    edges += find_exits

    print("Nodes:", len(nodes), "- Edges:", len(edges))
    for edge in edges:
        print(edge)

    robot_pos = localiseRobot()
    print("robot pose",robot_pos)
    # plotGraph(recMap, edges, wallpoints, robot_pos)
    plotNodePositionGraph(recMap, nodes, edges, wallpoints, robot_pos)

    # exemple node ([y, x])
    # new_robot_pos = [0, 0, 0]
    # for edge in edges:
    #     print(edge.parent.position)
    #     if(edge.parent.position == (22, 10)):
    #             new_robot_pos = goToNode(robot_pos, edge.parent, recMap)
    #             print("new_robot_pos", new_robot_pos)
    #     if(new_robot_pos != [0,0,0] and edge.parent.position == (22, 22)):
    #         goToNode(new_robot_pos, edge.parent, recMap)

if __name__ == "__main__":
    main()
