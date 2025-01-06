#!/usr/bin/env python3

import rospy
from map_utils import getMap, transformMap
from graph_utils import buildGraph, findExits, a_star_search, findClosestNode, Node
from visualization_utils import plotEvaluatedTrajectoriesGraph, plotMap, plotGraph, plotNodePositionGraph, plotArrowPathGraph, plotTransformGoadAndRobotPoseGraph
from robot_controller import PT2Block, evaluateControls, generateControls, localiseRobot, goToNode, transform_goal_relative_to_robot
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
    # plotNodePositionGraph(recMap, nodes, edges, wallpoints, robot_pos)

    start_node = findClosestNode(robot_pos, edges, recMap)
    
    global_path = [
    [2.5 ,1, 0],
    [3, 1.5, 0.5*np.pi],
    [2.5, 2, np.pi],
    [1.5, 2, np.pi],
    [1, 1.5, 1.5*np.pi],
    [1, 0.5, 1.5*np.pi],
    [0.5, 0, np.pi],
    [0, 0, np.pi],
    ]
    
    current_goal_ID = 1
    plotArrowPathGraph(robot_pos, global_path, wallpoints)
    goalpose = transform_goal_relative_to_robot(robot_pos,global_path[current_goal_ID])
    print("goal pose",goalpose)

    last_control = np.array([0, 0])
    controls = generateControls(last_control)
    
    ts = 1/2 # Sampling time [sec] -> 2Hz
    horizon = 10 # Number of time steps to simulate. 10*0.5 sec = 5 seconds lookahead into the future
    robotModelPT2 = PT2Block(ts=ts, T=0.05, D=0.8)
    costs, trajectories = evaluateControls(controls, robotModelPT2, 70, goalpose, ts)
    plotTransformGoadAndRobotPoseGraph(goalpose, wallpoints)

    plotEvaluatedTrajectoriesGraph(goalpose, wallpoints, trajectories, costs)
    
    # for pos in nodes:
    #     if start_node.position == nodes[pos].position:
    #         start_exits = True

    # if start_exits:
    #     paths = [a_star_search(grid, nodes, edges, start_node, exit.child) for exit in find_exits]

    #     print("Paths found:", [[step.position for step in path] for path in paths])

    #     # Get the path with the lower number of steps
    #     path = min(paths, key=lambda x: len(x)) if paths else None

    #     print("Path found:", [step.position for step in path])
    #     # Optional: Visualize path
    #     for step in path:
    #         print(f"Step: {step}")
    #         robot_pos = goToNode(robot_pos, step, recMap)
    # else:
    #     print("Start or goal node is invalid.")

    # Call plotArrowPath to visualize the global path

if __name__ == "__main__":
    main()
    