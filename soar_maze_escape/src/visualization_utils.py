import numpy as np
import matplotlib.pyplot as plt

from map_utils import convertMapToWorldCoordinates
from graph_utils import Node


# Store colors matching UAS TW colour scheme as dict 
COLOR_SCHEME = {
        "darkblue": "#143049",
        "twblue": "#00649C",
        "lightblue": "#8DA3B3",
        "lightgrey": "#CBC0D5",
        "twgrey": "#72777A"
    }


## Visualise transformed maze and scans
def plotMap(free_positions, wall_positions):
    # Create single figure
    plt.rcParams['figure.figsize'] = [7, 7]
    fig, ax = plt.subplots()

    # Plot data as points (=scatterplot) and label accordingly. The colours are to look nice with UAS TW colours
    ax.scatter(wall_positions[:,1], wall_positions[:,0], c=COLOR_SCHEME["darkblue"], alpha=1.0, s=6**2, label="Walls")
    ax.scatter(free_positions[:,1], free_positions[:,0], c=COLOR_SCHEME["twgrey"], alpha=0.08, s=6**2, label="Unobstructed Space")

    # Set axes labels and figure title
    ax.set_xlabel("X-Coordinate [m]")
    ax.set_ylabel("Y-Coordinate [m]")
    ax.set_title("Map Data Transformed into World Coordinates")

    # Set grid to only plot each metre
    ax.set_xticks = [-1, 0, 1, 2, 3, 4 ]
    ax.set_yticks = [-1, 0, 1, 2, 3, 4 ]

    # Move grid behind points
    ax.set_axisbelow(True)
    ax.grid()

    # Add labels
    ax.legend()

    # Show plot
    plt.show()


def plotGraph(recMap, edges, wall_positions, robot_pos):
    ## Visualise maze and generated graph
    # Create single figure
    plt.rcParams['figure.figsize'] = [7, 7]
    fig, ax = plt.subplots()

    # Get points on graph
    nodePositions = np.array([
        convertMapToWorldCoordinates(n.parent.position[0], n.parent.position[1], recMap) for n in edges
        ] + [
        convertMapToWorldCoordinates(n.child.position[0], n.child.position[1], recMap) for n in edges
    ])

    nodePositions = np.unique(nodePositions, axis=1)

    # Get lines connecting the nodes
    edgeLines = np.array(
        [
            [
                convertMapToWorldCoordinates(n.parent.position[0], n.parent.position[1], recMap),
                convertMapToWorldCoordinates(n.child.position[0], n.child.position[1], recMap)
            ] for n in edges
        ]
    )

    # Plot data as points (=scatterplot) and label accordingly. The colours are defined to look nice with UAS TW colours
    ax.scatter(wall_positions[:,1], wall_positions[:,0], c=COLOR_SCHEME["darkblue"], alpha=1.0, s=6**2, label="Walls")
    ax.scatter(nodePositions[:,1], nodePositions[:,0], c=COLOR_SCHEME["twblue"], alpha=1.0, s=8**2, label="Graph")
    ax.scatter([robot_pos[1]], [robot_pos[0]], c=COLOR_SCHEME["twblue"], s=15**2, label="Robot Position")

    # Plot lines connecting nodes
    for line in edgeLines:
        x0, y0 = line[0]
        x1, y1 = line[1]
        x = [x0, x1]
        y = [y0, y1]
        ax.plot(x, y, c=COLOR_SCHEME["twblue"])

    # Set axes labels and figure title
    ax.set_xlabel("X-Coordinate [m]")
    ax.set_ylabel("Y-Coordinate [m]")
    ax.set_title("Graph Generated based on Map Data")

    # Set grid to only plot each metre
    ax.set_xticks = [-1, 0, 1, 2, 3, 4 ]
    ax.set_yticks = [-1, 0, 1, 2, 3, 4 ]

    # Move grid behind points
    ax.set_axisbelow(True)
    ax.grid()

    # Add labels
    ax.legend()

    # Show plot
    plt.show()


def plotNodePositionGraph(recMap, nodes, edges, wall_positions, robot_pos):
    ## Visualise maze and generated graph
    # Create single figure
    plt.rcParams['figure.figsize'] = [7, 7]
    fig, ax = plt.subplots()

    # Convert node positions to world coordinates
    nodePositions = np.array([
        convertMapToWorldCoordinates(n[1], n[0], recMap) for n in nodes
    ])

    edgeLines = np.array(
        [
            [
                convertMapToWorldCoordinates(n.parent.position[0], n.parent.position[1], recMap),
                convertMapToWorldCoordinates(n.child.position[0], n.child.position[1], recMap)
            ] for n in edges
        ]
    )
    
    # Plot data as points (=scatterplot) and label accordingly. The colours are defined to look nice with UAS TW colours
    ax.scatter(wall_positions[:,1], wall_positions[:,0], c=COLOR_SCHEME["darkblue"], alpha=1.0, s=6**2, label="Walls")
    ax.scatter(nodePositions[:,1], nodePositions[:,0], c=COLOR_SCHEME["twblue"], alpha=1.0, s=8**2, label="Graph")

    for i, node in enumerate(nodes):
        x_pos = nodePositions[i][1]
        y_pos = nodePositions[i][0] + 0.1
        ax.text(x_pos, y_pos, f"{node}", fontsize=10, bbox=dict(facecolor=COLOR_SCHEME["lightblue"], alpha=0.5), color=COLOR_SCHEME["twblue"])
        
     # Plot lines connecting nodes
    for line in edgeLines:
        x0, y0 = line[0]
        x1, y1 = line[1]
        x = [x0, x1]
        y = [y0, y1]
        ax.plot(x, y, c=COLOR_SCHEME["twblue"])
    
    ax.scatter([robot_pos[1]], [robot_pos[0]], c=COLOR_SCHEME["twgrey"], s=15**2, label="Robot Position")


    # Set axes labels and figure title
    ax.set_xlabel("X-Coordinate [m]")
    ax.set_ylabel("Y-Coordinate [m]")
    ax.set_title("Graph Generated based on Map Data")

    # Set grid to only plot each metre
    ax.set_xticks = [-1, 0, 1, 2, 3, 4 ]
    ax.set_yticks = [-1, 0, 1, 2, 3, 4 ]

    # Move grid behind points
    ax.set_axisbelow(True)
    ax.grid()

    # Add labels
    ax.legend()

    # Show plot
    plt.show()
    
    
    
def plotArrowPathGraph(robotpose, global_path, wallPositions):
    plt.rcParams['figure.figsize'] = [7, 7]
    fig, ax = plt.subplots()

    positions_on_path = np.array([robotpose.tolist()] + global_path)[:,:2]
    ax.plot(positions_on_path[:,0], positions_on_path[:,1], c=COLOR_SCHEME["lightblue"], alpha=0.6, label="Global Path")

    # Plot robot poses as arrows to indicate orientation
    for s in [robotpose.tolist()] + global_path:
        # Calculate arrow's dx and dy from s = [x, y, theta]
        p2 = np.array([np.cos(s[2]), np.sin(s[2])])
        # Scale arrow length and draw
        p2 /= np.linalg.norm(p2)*4
        ax.arrow(s[0], s[1], p2[0], p2[1], width=0.02)

    # Plot walls (from Step 1, if the variable exists)
    if 'wallPositions' in locals():
        try: ax.scatter(wallPositions[:,1], wallPositions[:,0], c=COLOR_SCHEME["darkblue"], alpha=1.0, s=6**2, label="Walls")
        except: pass

    # Set axes labels and figure title
    ax.set_xlabel("X-Coordinate [m]")
    ax.set_ylabel("Y-Coordinate [m]")
    ax.set_title("Global Path Resulting from Step 2")

    # Set grid to only plot each metre
    ax.set_xticks = [-1, 0, 1, 2, 3, 4 ]
    ax.set_yticks = [-1, 0, 1, 2, 3, 4 ]
    ax.set_xlim(-1, 4)
    ax.set_ylim(-1, 4)

    # Move grid behind points
    ax.set_axisbelow(True)
    ax.grid()

    # Add labels
    ax.legend()

    # Show plot
    plt.show()
    
    
def plotTransformGoadAndRobotPoseGraph(goalpose, wallPositions):
    ## Visualise transformed goal and robot poses
    # Create single figure
    plt.rcParams['figure.figsize'] = [7, 7]
    fig, ax = plt.subplots()

    # Plot robot poses as arrows to indicate orientation
    poses = [[0,0,0], goalpose.tolist()]
    labels = ["Robot Pose", "Goal Pose"]
    colours = [COLOR_SCHEME["lightblue"], COLOR_SCHEME["darkblue"]]

    for s, l, c in zip(poses, labels, colours):
        # Calculate arrow's dx and dy from s = [x, y, theta]
        p2 = np.array([np.cos(s[2]), np.sin(s[2])])
        # Scale arrow length and draw
        p2 /= np.linalg.norm(p2)*4
        ax.arrow(s[0], s[1], p2[0], p2[1], width=0.02, label=l, color=c)

    # Plot walls (from Step 1, if the variable exists)
    if 'wallPositions' in locals():
        try: ax.scatter(wallPositions[:,1], wallPositions[:,0], c=COLOR_SCHEME["darkblue"], alpha=1.0, s=6**2, label="Walls")
        except: pass

    # Set axes labels and figure title
    ax.set_xlabel("X-Coordinate (Robot Coordinate System) [m]")
    ax.set_ylabel("Y-Coordinate (Robot Coordinate System [m]")
    ax.set_title("First Goal Transformed to Robot Coordinate System")

    # Set grid to only plot each metre
    ax.set_xticks = [-1, 0, 1]
    ax.set_yticks = [-1, 0, 1]
    ax.set_xlim(-0, 1.5)
    ax.set_ylim(-0.5, 1)

    # Move grid behind points
    ax.set_axisbelow(True)
    ax.grid()

    # Add labels
    ax.legend()

    # Show plot
    plt.show()