import rospy
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def localiseRobot():
    """Localises the robot towards the 'map' coordinate frame. Returns pose in format (x,y,theta)"""
    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer) # DO not remove this line, location function crash otherwise

    while True:
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Robot localisation took longer than 1 sec")
            continue

    theta = R.from_quat([
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w]).as_euler("xyz")[2]

    rospy.loginfo(f"Position du robot : x={trans.transform.translation.x}, y={trans.transform.translation.y}, theta={theta}")
    return trans.transform.translation.y, trans.transform.translation.x,  theta


def publish_velocity(linear, angular):
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    cmd_vel_pub.publish(twist)
    

def rotateTowardTarget(desired_theta, theta_robot):
    delta_theta = desired_theta - theta_robot
    delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi

    while abs(delta_theta) > 0.01:
        publish_velocity(0.0, 0.5 * delta_theta)
        rospy.sleep(0.1)
        current_pose = localiseRobot()
        _, _, theta_robot = current_pose
        delta_theta = desired_theta - theta_robot
        delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi


def markerToDebug(y_target, x_target):
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "target_node"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = Point(x_target, y_target, 0)
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    rospy.sleep(5)
    marker_pub.publish(marker)


def goToNode(current_pose, target_node, recMap):
    y_r, x_r, theta_r = current_pose
    y_node, x_node = target_node.get_position()
    y_target, x_target = convertMapToWorldCoordinates(y_node, x_node, recMap)
    
    markerToDebug(y_target, x_target)

    # Calculate the desired angle to the target and rotate the robot accordingly
    theta_desired = np.arctan2(y_target - y_r, x_target - x_r)
    delta_theta = theta_desired - theta_r
    delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi

    rotateTowardTarget(theta_desired, theta_r)
    
    # Go to the target node
    dy = y_target - y_r
    dx = x_target - x_r
    distance = np.sqrt(np.square(dy) + np.square(dx))

    while distance > 0.1:
        linear_velocity = min(distance * 0.5, 0.3)
        publish_velocity(linear_velocity, 0)
        rospy.sleep(0.1)
        current_pose = localiseRobot()
        y_r, x_r, _ = current_pose
        distance = np.sqrt(np.square(y_target - y_r) + np.square(x_target - x_r))

    # Stopper le robot
    publish_velocity(0.0, 0.0)
    rospy.loginfo("Robot arrived at target node")


