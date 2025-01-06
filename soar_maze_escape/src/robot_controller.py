import rospy
import numpy as np
import numpy.typing as npt
import tf2_ros
import numpy.typing as npt
import copy
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from map_utils import convertMapToWorldCoordinates

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
    # return trans.transform.translation.y, trans.transform.translation.x,  theta
    return np.array([
        trans.transform.translation.x,
        trans.transform.translation.y,
        theta])

def pose2tf_mat(pose):
    """Converts a pose (x, y, theta) into a homogeneous transformation matrix"""
    x, y, theta = pose
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    return np.array([
        [cos_theta, -sin_theta, x],
        [sin_theta, cos_theta, y],
        [0, 0, 1]
    ])

def tf_mat2pose(tf_mat):
    """Converts a homogeneous transformation matrix into a pose (x, y, theta)"""
    x = tf_mat[0, 2]
    y = tf_mat[1, 2]
    theta = np.arctan2(tf_mat[1, 0], tf_mat[0, 0])
    return np.array([x, y, theta])
    
    
def transform_goal_relative_to_robot(robot_pose, goal_pose):
    """Transforms the goal pose to be relative to the robot pose."""
    robot_tf = pose2tf_mat(robot_pose)
    goal_tf = pose2tf_mat(goal_pose)
    
    robot_tf_inv = np.linalg.inv(robot_tf)
    
    relative_tf = np.dot(robot_tf_inv, goal_tf)
    
    relative_pose = tf_mat2pose(relative_tf)
    return relative_pose

def forwardKinematics(control: npt.ArrayLike, lastPose: npt.ArrayLike, dt: float, dtype=np.float64) -> np.ndarray:
    """Mobile robot forward kinematics (see Thrun Probabilistic Robotics)
    """
    if not isinstance(lastPose, np.ndarray):  # Check input formatting
        lastPose = np.array(lastPose, dtype=dtype)
    assert lastPose.shape == (3,), "Wrong pose format. Pose must be provided as list or array of form [x, y, theta]"
    if not isinstance(control, np.ndarray): control = np.array(control)
    assert control.shape == (2,), "Wrong control format. Control must be provided as list or array of form [vt, wt]"
    vt, wt = control
    # Set omega to smallest possible nonzero value in case it is zero to avoid division by zero
    if wt == 0: wt = np.finfo(dtype).tiny
    vtwt = vt/wt
    _, _, theta = lastPose
    return lastPose + np.array([
        -vtwt*np.sin(theta) + vtwt*np.sin(theta + (wt*dt)),
        vtwt*np.cos(theta) - vtwt*np.cos(theta + (wt*dt)),
        wt*dt
    ], dtype=dtype)

class PT2Block:
    """Discrete PT2 Block approximated using the Tustin approximation (rough robot dynamics model)
    """
    def __init__(self, T=0, D=0, kp=1, ts=0, bufferLength=3) -> None:
        self.k1, self.k2, self.k3, self.k4, self.k5, self.k6 = 0, 0, 0, 0, 0, 0
        self.e = [0 for i in range(bufferLength)]
        self.y = [0 for i in range(bufferLength)]
        if ts != 0:  self.setConstants(T, D, kp, ts)
    #
    def setConstants(self, T, D, kp, ts) -> None:
        self.k1 = 4*T**2 + 4*D*T*ts + ts**2
        self.k2 = 2*ts**2 - 8*T**2
        self.k3 = 4*T**2 - 4*D*T*ts + ts**2
        self.k4 = kp*ts**2
        self.k5 = 2*kp*ts**2
        self.k6 = kp*ts**2
    #
    def update(self, e) -> float:    
        self.e = [e]+self.e[:len(self.e)-1] # Update buffered input and output signals
        self.y = [0]+self.y[:len(self.y)-1]
        e, y = self.e, self.y # Shorten variable names for better readability
        # Calculate output signal and return output
        y[0] = ( e[0]*self.k4 + e[1]*self.k5 + e[2]*self.k6 - y[1]*self.k2 - y[2]*self.k3 )/self.k1
        return y[0]

def generateControls(lastControl: npt.ArrayLike) -> np.ndarray:
    """
    Generates a list of possible (vt, wt) pairs that lead the robot towards the next goal.
    The generated control signals do not deviate too far from the last applied control.
    """
    vt_min, vt_max = -0.025, 0.025  # Min and max linear velocities
    wt_min, wt_max = -1.4, 1.4  # Min and max angular velocities
    vt_step, wt_step = 0.0115, 0.025  # Steps for linear and angular velocities

    vt_range = np.arange(vt_min, vt_max + vt_step, vt_step)
    wt_range = np.arange(wt_min, wt_max + wt_step, wt_step)

    controls = np.array([[vt, wt] for vt in vt_range for wt in wt_range])

    # Filter controls to ensure they do not deviate too far from the last applied control
    max_vt_deviation = 0.05
    max_wt_deviation = 1.4

    valid_controls = controls[
        (np.abs(controls[:, 0] - lastControl[0]) <= max_vt_deviation) &
        (np.abs(controls[:, 1] - lastControl[1]) <= max_wt_deviation)
    ]

    return valid_controls

def costFn(pose: npt.ArrayLike, goalpose: npt.ArrayLike, control: npt.ArrayLike) -> float:
    """Calculates the cost based on the pose"""

    error = np.abs(pose - goalpose)
    
    # handle rotation errors that are <−π or >π
    error[2] = np.mod(error[2] + np.pi, 2 * np.pi) - np.pi

    # State weighting matrix Q
    Q = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.5]
    ])

    # Control weighting matrix R
    R = np.array([
        [0.1, 0.0],
        [0.0, 0.1]
    ])

    eT_Q_e = np.dot(error.T, np.dot(Q, error))

    control = np.abs(control)

    uT_R_u = np.dot(control.T, np.dot(R, control))

    cost = eT_Q_e + uT_R_u
    
    return cost

def evaluateControls(controls, robotModelPT2, horizon, goalpose, ts):
    costs = np.zeros_like(np.array(controls)[:,0], dtype=float)
    trajectories = [ [] for _ in controls ]
    
    # Apply range of control signals and compute outcomes
    for ctrl_idx, control in enumerate(controls):
    
        # Copy currently predicted robot state
        forwardSimPT2 = copy.deepcopy(robotModelPT2)
        forwardpose = [0,0,0]
    
        # Simulate until horizon
        for step in range(horizon):
            control_sim = copy.deepcopy(control)
            v_t, w_t = control
            v_t_dynamic = forwardSimPT2.update(v_t)
            control_dym = [v_t_dynamic, w_t]
            forwardpose = forwardKinematics(control_dym, forwardpose, ts)
            costs[ctrl_idx] += costFn(forwardpose, goalpose, control_sim)
            # Track trajectory for visualisation
            trajectories[ctrl_idx].append(forwardpose)

    return costs, trajectories

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
    x_node, y_node  = target_node.position
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
    return current_pose


