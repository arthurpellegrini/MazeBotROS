# MazeBotROS - Maze Escape Project - Turtlebot ROS Simulation

This project is part of the **Mobile Robotics Course** at UAS Technikum Wien. The goal is to create an autonomous navigation solution for a Turtlebot 3 robot to escape a maze. The project includes ROS nodes for **global path planning** and **local control** to guide the robot from its starting position to one of the maze exits. 

## Project Overview

The maze simulation uses a provided ROS package that initializes a Turtlebot within a maze environment. The project requires implementing nodes with the following functionalities:
- **Global Planning Node**: Reads the Turtlebot’s current position and calculates a path to a maze exit.
- **Local Control Node**: Executes the path by controlling the Turtlebot's velocity, adjusting its course as necessary to follow the planned path.

### Key Objectives
- Navigate the Turtlebot to a maze exit from any specified starting location.
- Implement a non-hardcoded search algorithm based on the ROS-provided map.
- Develop a local control solution to execute the planned path using a cost function and forward simulation techniques.

## Project Structure
The project consists of the following files and directories:
- **src/**: Contains the ROS nodes for global planning and local control.
- **launch/**: Launch files for initializing the maze simulation (`launchSim.launch`).
- **scripts/**: Additional scripts and tools.
- **notebooks/**: Jupyter Notebook with the project’s data processing and visualization (pending downloadable access).

## Getting Started

1. **Prerequisites**:
   - ROS Noetic (or compatible ROS distribution).
   - Turtlebot 3 Simulation packages.
   - Python 3 for executing Jupyter notebooks.
2. **Installation**:
   - Clone this repository.
   - Ensure the necessary ROS dependencies are installed.
   - Launch the maze simulation using `launchSim.launch`.

3. **Running the Simulation**:
   - Start the simulation with `roslaunch launchSim.launch`.
   - Execute the global planning and local control nodes.

## Cookbook Notebook
A Jupyter Notebook (`Project Cookbook`) has been provided to demonstrate the project’s functionality, including data processing and visualization steps. **Note**: Currently, the notebook is only viewable in Moodle. We are working on obtaining a downloadable version for easier access.

## Project Requirements
- Implemented ROS Package(s).
- Documentation of the ROS nodes and overall architecture.
- Flowchart detailing the navigation and control logic.
- Presentation slides for the final project presentation.
