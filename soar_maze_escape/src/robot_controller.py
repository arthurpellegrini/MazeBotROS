#!/usr/bin/env python3
import rospy
import numpy as np
import numpy.typing as npt
import tf2_ros
import numpy.typing as npt
import copy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path


# Publishers
CMD_VEL_PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
TRAJECTORY_PUB = rospy.Publisher('/trajectory', Path, queue_size=10)
GOAL_PUB = rospy.Publisher('/goal_pose', PoseStamped, queue_size=10)


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


def convertPose2TfMatrix(pose):
    """Converts a pose (x, y, theta) into a homogeneous transformation matrix"""
    x, y, theta = pose
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    return np.array([
        [cos_theta, -sin_theta, x],
        [sin_theta, cos_theta, y],
        [0, 0, 1]
    ])


def convertTfMatrix2Pose(tf_mat):
    """Converts a homogeneous transformation matrix into a pose (x, y, theta)"""
    x = tf_mat[0, 2]
    y = tf_mat[1, 2]
    theta = np.arctan2(tf_mat[1, 0], tf_mat[0, 0])
    return np.array([x, y, theta])


def transformGoalRelativeToRobot(robot_pose, goal_pose):
    """Transforms the goal pose to be relative to the robot pose."""
    robot_tf = convertPose2TfMatrix(robot_pose)
    goal_tf = convertPose2TfMatrix(goal_pose)
    
    robot_tf_inv = np.linalg.inv(robot_tf)
    
    relative_tf = np.dot(robot_tf_inv, goal_tf)
    
    relative_pose = convertTfMatrix2Pose(relative_tf)
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


def pubCMD(control):
    """Publishes the control input as a geometry_msgs/Twist message to the /cmd_vel topic"""
    twist = Twist()
    twist.linear.x = control[0] 
    twist.angular.z = control[1]
    CMD_VEL_PUB.publish(twist)


def pubTrajectory(trajectory):
    """Publishes the trajectory as a nav_msgs/Path message to the /trajectory topic"""
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "base_link"

    for pose in trajectory:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.orientation.z = np.sin(pose[2] / 2.0)
        pose_msg.pose.orientation.w = np.cos(pose[2] / 2.0)
        path_msg.poses.append(pose_msg)

    TRAJECTORY_PUB.publish(path_msg)


def pubGoal(goalpose):
    """Publishes the goalpose as a geometry_msgs/PoseStamped message to the /goal_pose topic"""
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "base_link"
    pose_msg.pose.position.x = goalpose[0]
    pose_msg.pose.position.y = goalpose[1]
    pose_msg.pose.orientation.z = np.sin(goalpose[2] / 2.0)
    pose_msg.pose.orientation.w = np.cos(goalpose[2] / 2.0)
    GOAL_PUB.publish(pose_msg)
