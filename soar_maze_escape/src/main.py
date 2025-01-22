#!/usr/bin/env python3
import rospy
from map_utils import getMap, transformMap
from graph_utils import buildGraph, findExits, findClosestNode, reconstructBestPath
from visualization_utils import plotMap, plotGraph, plotNodePositionGraph, plotArrowPathGraph, plotTransformGoadAndRobotPoseGraph, plotEvaluatedTrajectoriesGraph
from robot_controller import localiseRobot, generateControls, pubCMD, pubGoal, pubTrajectory, PT2Block, evaluateControls, transformGoalRelativeToRobot
import numpy as np


# DEBUG FLAGS
DEBUG_MAP = False
DEBUG_GRAPH = False
DEBUG_PATH = False
DEBUG_TRAJECTORY = False


def main():
    """
    Main function to initialize the ROS node and execute the maze navigation algorithm.
    """
    rospy.init_node("moro_maze_navigation")

    recMap = getMap()
    freepoints, wallpoints = transformMap(recMap)

    if DEBUG_MAP:
        plotMap(freepoints, wallpoints)

    # Convert map to 2D grid
    grid = np.array(recMap.data).reshape((recMap.info.height, recMap.info.width))

    # Build graph
    nodes, edges = buildGraph(grid)

    # Add exits to the graph
    nodes, find_exits = findExits(grid, nodes)
    edges += find_exits

    robot_pose = localiseRobot()

    if DEBUG_GRAPH:
        # plotGraph(recMap, edges, wallpoints, robot_pose)
        # We will use the second one because the first plot is include inside the plotNodePositionGraph function
        plotNodePositionGraph(recMap, nodes, edges, wallpoints, robot_pose)

    # A* search
    start_node = findClosestNode(robot_pose, edges, recMap)
    global_path = reconstructBestPath(nodes, start_node, edges, find_exits, grid, recMap)

    if DEBUG_PATH:
        plotArrowPathGraph(robot_pose, global_path, wallpoints)

    # Parameters - TODO: Tune these parameters
    ts = 0.5
    horizon = 70
    robot_model = PT2Block(ts=ts, T=0.05, D=0.8)
    last_control = np.array([0, 0]) 
    current_goal_ID = 1
    trigger_distance = 0.2

    while not rospy.is_shutdown():
        robot_pose = localiseRobot()
        goal_pose = transformGoalRelativeToRobot(robot_pose,global_path[current_goal_ID])
        controls = generateControls(last_control)
        costs, trajectories = evaluateControls(controls, robot_model, horizon, goal_pose, ts)
        best_idx = np.argmin(costs)
        last_control = controls[best_idx]

        if DEBUG_TRAJECTORY:
            # plotTransformGoadAndRobotPoseGraph(goal_pose, wallpoints)
            # We will use the second one because the first plot is include inside the evaluateTrajectoriesGraph function
            plotEvaluatedTrajectoriesGraph(goal_pose, wallpoints, trajectories, costs)

        pubCMD(last_control)
        pubTrajectory(trajectories[best_idx])
        pubGoal(goal_pose)

        # Stop if goal reached
        if np.linalg.norm(goal_pose[:2]) < trigger_distance:
            current_goal_ID += 1

            if current_goal_ID == len(global_path):
                break
        
    # Stop robot
    pubCMD([0, 0])


if __name__ == "__main__":
    main()
