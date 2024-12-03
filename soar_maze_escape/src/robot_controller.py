import rospy
import numpy as np
from geometry_msgs.msg import Twist
import tf2_ros

def get_robot_position():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1)
    transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
    x = transform.transform.translation.x
    y = transform.transform.translation.y
    q = transform.transform.rotation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
    theta = np.arctan2(siny_cosp, cosy_cosp)
    return x, y, theta

def world_to_grid(world_x, world_y, resolution, origin):
    """Converts world coordinates to grid coordinates."""
    grid_x = int((world_x - origin.x) / resolution)
    grid_y = int((world_y - origin.y) / resolution)
    return grid_x, grid_y


def grid_to_world(grid_x, grid_y, resolution, origin):
    """Converts grid coordinates to world coordinates."""
    world_x = grid_x * resolution + origin.x
    world_y = grid_y * resolution + origin.y
    return world_x, world_y


def align_and_move(waypoint_x, waypoint_y, cmd_vel_pub):
    """Aligns the robot to the waypoint and moves towards it."""
    robot_x, robot_y, robot_theta = get_robot_position()
    dx = waypoint_x - robot_x
    dy = waypoint_y - robot_y
    distance = np.sqrt(dx**2 + dy**2)

    # Align robot
    target_angle = np.arctan2(dy, dx)
    angular_diff = target_angle - robot_theta
    angular_diff = np.arctan2(np.sin(angular_diff), np.cos(angular_diff))  # Normalize to [-pi, pi]

    # Aligning loop
    while abs(angular_diff) > 0.1:  # Threshold for alignment (radians)
        angular_velocity = max(min(angular_diff * 2.0, 1.0), -1.0)  # Limit angular speed
        publish_velocity(0, angular_velocity, cmd_vel_pub)  # Rotate towards target
        rospy.sleep(0.1)

        # Update robot's orientation dynamically
        _, _, robot_theta = get_robot_position()
        angular_diff = target_angle - robot_theta
        angular_diff = np.arctan2(np.sin(angular_diff), np.cos(angular_diff))  # Normalize again

    publish_velocity(0, 0, cmd_vel_pub)  # Stop rotation

    # Move robot
    while distance > 0.1:  # Threshold for reaching the waypoint
        robot_x, robot_y, _ = get_robot_position()
        dx = waypoint_x - robot_x
        dy = waypoint_y - robot_y
        distance = np.sqrt(dx**2 + dy**2)
        linear_velocity = min(distance * 0.5, 0.3)  # Cap max speed
        publish_velocity(linear_velocity, 0, cmd_vel_pub)  # Move forward
        rospy.sleep(0.1)

    publish_velocity(0, 0, cmd_vel_pub)  # Stop robot


def publish_velocity(linear, angular, cmd_vel_pub):
    """Publishes velocity commands to the robot."""
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    cmd_vel_pub.publish(twist)


def follow_path(path, resolution, origin, cmd_vel_pub):
    for grid_point in path:
        waypoint_x, waypoint_y = grid_to_world(grid_point[0], grid_point[1], resolution, origin)
        align_and_move(waypoint_x, waypoint_y, cmd_vel_pub)
