import numpy as np
import matplotlib.pyplot as plt

from map_utils import convertMapToWorldCoordinates


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

