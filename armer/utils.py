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
    
def mjtg(robot: rtb.ERobot, qd: np.ndarray, max_speed: float=0.2, max_rot: float=0.5, frequency=500):
  # This is the average cartesian speed we want the robot to move at
  # NOTE: divided by 2 to make the max speed the approx. peak of the speed achieved
  ave_cart_speed = max_speed / 2
      
  # Calculate start and end pose linear distance to estimate the expected time
  current_ee_mat = robot.ets(start=robot.base_link, end=robot.gripper).eval(robot.q)
  mid_ee_mat = robot.ets(start=robot.base_link, end=robot.gripper).eval(qd - (qd - robot.q) / 2)
  end_ee_mat = robot.ets(start=robot.base_link, end=robot.gripper).eval(qd)
  
  current_ee_pose = current_ee_mat[:3, 3]
  mid_ee_pose = mid_ee_mat[:3, 3]
  end_ee_pose = end_ee_mat[:3, 3]
  
  # Estimation of time taken based on linear motion from current to end cartesian pose
  # We may require some optimisation of this given the curved nature of the actual ee trajectory
  D1 = np.sqrt((mid_ee_pose[0] - current_ee_pose[0])**2 +
      (mid_ee_pose[1] - current_ee_pose[1])**2 +
      (mid_ee_pose[2] - current_ee_pose[2])**2)
  
  D2 = np.sqrt((end_ee_pose[0] - mid_ee_pose[0])**2 +
      (end_ee_pose[1] - mid_ee_pose[1])**2 +
      (end_ee_pose[2] - mid_ee_pose[2])**2)

  linear_move_time = (D1 + D2) / ave_cart_speed

  current_ee_rot = current_ee_mat[:3,:3]
  end_ee_rot = end_ee_mat[:3,:3]

  angular_move_time = np.arccos((np.trace(np.transpose(end_ee_rot) @ current_ee_rot) - 1) / 2) / max_rot
  move_time = max(linear_move_time, angular_move_time)

  # Move time correction [currently un-used but requires optimisation]
  # Correction to account for error in curved motion
  move_time = move_time * 1

  # Obtain minimum jerk velocity profile of joints based on estimated end effector move time
  dq = []
  dqd = []

  timefreq = int(move_time * frequency)

  for time in range(1, timefreq):
    dq.append(
        robot.q + (qd - robot.q) *
        (10.0 * (time/timefreq)**3
        - 15.0 * (time/timefreq)**4
        + 6.0 * (time/timefreq)**5))

    dqd.append(
        frequency * (1.0/timefreq) * (qd - robot.q) *
        (30.0 * (time/timefreq)**2.0
        - 60.0 * (time/timefreq)**3.0
        + 30.0 * (time/timefreq)**4.0))

  # Calculate time frequency - based on the max time required for trajectory and the frequency of operation
  time_freq_steps = int(move_time * frequency)
  
  return Trajectory('minimum-jerk', move_time, dq, dqd, None, True)

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
