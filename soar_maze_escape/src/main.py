#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from map_utils import get_map, transform_map, find_exits
from graph_utils import create_graph, a_star_search
from robot_controller import follow_path, get_robot_position, world_to_grid
from visualization_utils import publish_path_marker, plot_map_with_path


def main():
    rospy.init_node('local_path_follower')

    global cmd_vel_pub, path_marker_pub

    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    path_marker_pub = rospy.Publisher("/a_star_path", Marker, queue_size=10)

    rospy.sleep(5)  # Allow publishers to initialize

    try:
        occupancy_grid = get_map()
        grid, resolution, origin = transform_map(occupancy_grid)
        graph = create_graph(grid)

        robot_x, robot_y, robot_theta = get_robot_position()
        start = world_to_grid(robot_x, robot_y, resolution, origin)

        exits = find_exits(grid)
        paths = [a_star_search(graph, start, e) for e in exits]
        valid_paths = [path for path in paths if path]

        if not valid_paths:
            rospy.logwarn("No valid paths found to any exit.")
            return
        
        shortest_path = min(valid_paths, key=len)
        plot_map_with_path(grid, shortest_path, resolution, origin)
        publish_path_marker(shortest_path, resolution, origin, path_marker_pub)
        follow_path(shortest_path, resolution, origin, cmd_vel_pub)

        rospy.loginfo("Robot has exited the maze.")
    except Exception as e:
        rospy.logerr(f"Error: {e}")

if __name__ == "__main__":
    main()
