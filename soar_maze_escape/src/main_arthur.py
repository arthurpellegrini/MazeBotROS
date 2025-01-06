#!/usr/bin/env python3

import rospy
from map_utils import getMap, transformMap
from graph_utils import buildGraph, findExits, findClosestNode, a_star_search, path_reconstrcution
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

    # A* search
    start_node = findClosestNode(robot_pos, edges, recMap)

    for pos in nodes:
        if start_node.position == nodes[pos].position:
            start_exits = True

    if start_exits:
        paths = [a_star_search(grid, nodes, edges, start_node, exit.child) for exit in find_exits]
        print("Paths found:", [[step.position for step in path] for path in paths])
        # Get the path with the lower number of steps
        path = min(paths, key=lambda x: len(x)) if paths else None
        print("Path found:", [step.position for step in path])
        global_path = path_reconstrcution(path)
        print("Global path:", global_path)
    else:
        print("Start or goal node is invalid.")


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
