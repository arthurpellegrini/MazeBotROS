#!/usr/bin/env python3

import rospy
from map_utils import getMap, transformMap
from visualization_utils import plotMap

def main():
    # Initiate ROS node
    rospy.init_node('moro_maze_navigation')
    recMap = getMap()

    freepoints, wallpoints = transformMap(recMap)
    plotMap(freepoints, wallpoints)

if __name__ == "__main__":
    main()
