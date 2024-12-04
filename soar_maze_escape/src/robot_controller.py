import rospy
import numpy as np
from geometry_msgs.msg import Twist
import tf2_ros
from scipy.spatial.transform import Rotation as R

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

