import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
import numpy as np


def plotMap(free_positions, wall_positions):
    # Store colours matching UAS TW colour scheme as dict 
    colour_scheme = {
        "darkblue": "#143049",
        "twblue": "#00649C",
        "lightblue": "#8DA3B3",
        "lightgrey": "#CBC0D5",
        "twgrey": "#72777A"
    }

    ## Visualise transformed maze and scans
    # Create single figure
    plt.rcParams['figure.figsize'] = [7, 7]
    fig, ax = plt.subplots()

    # Plot data as points (=scatterplot) and label accordingly. The colours are to look nice with UAS TW colours
    ax.scatter(wall_positions[:,1], wall_positions[:,0], c=colour_scheme["darkblue"], alpha=1.0, s=6**2, label="Walls")
    ax.scatter(free_positions[:,1], free_positions[:,0], c=colour_scheme["twgrey"], alpha=0.08, s=6**2, label="Unobstructed Space")

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
