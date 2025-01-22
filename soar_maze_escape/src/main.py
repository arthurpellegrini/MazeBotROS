#!/usr/bin/env python3
import rospy
from map_utils import getMap, transformMap
from graph_utils import buildGraph, findExits, findClosestNode, reconstructBestPath
from visualization_utils import plotMap, plotGraph, plotNodePositionGraph, plotArrowPathGraph, plotTransformGoadAndRobotPoseGraph, plotEvaluatedTrajectoriesGraph
from robot_controller import localiseRobot, generateControls, pubCMD, pubGoal, pubTrajectory, PT2Block, evaluateControls, transformGoalRelativeToRobot
import numpy as np

def main():
    rospy.init_node("moro_maze_navigation")

    recMap = getMap()
    freepoints, wallpoints = transformMap(recMap)

    # DEBUG - Plot map
    plotMap(freepoints, wallpoints)

    # Convert map to 2D grid
    grid = np.array(recMap.data).reshape((recMap.info.height, recMap.info.width))

    # Build graph
    nodes, edges = buildGraph(grid)

    # Add exits to the graph
    nodes, find_exits = findExits(grid, nodes)
    edges += find_exits

    robot_pose = localiseRobot()

    # DEBUG - Plot graph
    plotGraph(recMap, edges, wallpoints, robot_pose)
    plotNodePositionGraph(recMap, nodes, edges, wallpoints, robot_pose)

    # A* search
    start_node = findClosestNode(robot_pose, edges, recMap)
    global_path = reconstructBestPath(nodes, start_node, edges, find_exits, grid, recMap)

    # DEBUG - Plot path
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

        # DEBUG - Plot goal pose and evaluated trajectories
        plotTransformGoadAndRobotPoseGraph(goal_pose, wallpoints)
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
    