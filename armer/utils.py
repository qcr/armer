"""
Utility functions used by Armer

.. codeauthor:: Gavin Suddrey
.. codeauthor:: Dasun Gunasinghe
"""

import rospy
import numpy as np
from spatialmath import SE3, SO3, UnitQuaternion, base
from geometry_msgs.msg import TransformStamped
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import Trajectory
from scipy.interpolate import interp1d

def ikine(robot, target, q0, end):
    Tep = SE3(target.position.x, target.position.y, target.position.z) * \
            UnitQuaternion(target.orientation.w, [target.orientation.x, target.orientation.y, target.orientation.z]).SE3()
            
    # Using the roboticstoolbox Levemberg-Marquadt (LM) Numerical inverse kinematics solver
    result = robot.ik_LM(
        Tep,
        end=end,
        q0=np.array(q0)
    )
    
    if not (result[1]):
        rospy.logerr('Unable to generate inverse kinematic solution')
        return type('obj', (object,), {'q' : q0})

    return type('obj', (object,), {'q' : np.array(result[0])})
    
def mjtg(robot: rtb.Robot, qf: np.ndarray, max_speed: float=0.2, max_rot: float=0.5, frequency=500):
    # This is the average cartesian speed we want the robot to move at
    # NOTE: divided by approx. 2 to make the max speed the approx. peak of the speed achieved
    # TODO: investigate a better approximation strategy here
    ave_cart_speed = max_speed / 1.92

    start_SE3 = SE3(robot.ets(start=robot.base_link, end=robot.gripper).eval(robot.q))
    end_SE3 = SE3(robot.ets(start=robot.base_link, end=robot.gripper).eval(qf))

    # Compute the quintic polynomial scalar representation of trajectory (in cartesian space)
    ctraj = rtb.tools.trajectory.ctraj(start_SE3, end_SE3, frequency)
    D_sum = 0
    for i in range(frequency):
        if i < frequency-1:
            delta = ctraj[i].delta(ctraj[i+1])
            D = np.sqrt((delta[0]**2) + (delta[1]**2) + (delta[2]**2))
            D_sum += D

    linear_move_time = D_sum / ave_cart_speed
    angular_move_time = np.arccos((np.trace(np.transpose(end_SE3.R) @ start_SE3.R) - 1) / 2) / max_rot
    move_time = max(linear_move_time, angular_move_time)

    # DEBUGGING
    # print(f"linear time: {linear_move_time} | angular time: {angular_move_time} | move_time: {move_time}")

    # Obtain minimum jerk velocity profile of joints based on estimated end effector move time
    qd = []
    qdd = []

    # Calculate time frequency - based on the max time required for trajectory and the frequency of operation
    timefreq = int(move_time * frequency)
    for time in range(1, timefreq):
        qd.append(
            robot.q + (qf - robot.q) *
            (10.0 * (time/timefreq)**3
            - 15.0 * (time/timefreq)**4
            + 6.0 * (time/timefreq)**5))

        qdd.append(
            frequency * (1.0/timefreq) * (qf - robot.q) *
            (30.0 * (time/timefreq)**2.0
            - 60.0 * (time/timefreq)**3.0
            + 30.0 * (time/timefreq)**4.0))
    
    return Trajectory('minimum-jerk', move_time, qd, qdd, None, True)

def populate_transform_stamped(parent_name: str, link_name: str, transform: np.array):
    """
    Generates a geometry_msgs/TransformStamped message between
    a link and its parent based on the provided SE3 transform
    """
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = parent_name
    transform_stamped.child_frame_id = link_name
    transform_stamped.transform.translation.x = transform[0,3]
    transform_stamped.transform.translation.y = transform[1,3]
    transform_stamped.transform.translation.z = transform[2,3]

    rot = base.r2q(transform[:3,:3])
    
    transform_stamped.transform.rotation.w = rot[0]
    transform_stamped.transform.rotation.x = rot[1]
    transform_stamped.transform.rotation.y = rot[2]
    transform_stamped.transform.rotation.z = rot[3]

    return transform_stamped
