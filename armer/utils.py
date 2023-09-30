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
    # NOTE: takes into account joint limits of robot (qlim)
    result = robot.ik_LM(
        Tep,
        end=end,
        q0=np.array(q0)
    )
    
    if not (result[1]):
        rospy.logerr('Unable to generate inverse kinematic solution')
        return type('obj', (object,), {'q' : q0})

    return type('obj', (object,), {'q' : np.array(result[0])})

def cartesian_move_time(robot: rtb.Robot, qf: np.ndarray, max_speed: float=0.2, max_rot: float=0.5, frequency=500):
    """
    Computes the Cartesian movement time given a start and end joint space state 
    """
    pass

def trapezoidal(robot: rtb.Robot, qf: np.ndarray, max_speed: float=0.2, frequency=500, move_time_sec: float=5, linear: bool=True):
    rospy.loginfo(f"TESTING Hotfix 96fd293")

    # ------------ NOTE: determine a joint trajectory (curved) or Cartesian (straight) based on request
    # Solve a trapezoidal trajectory in joint space based on provided solution based on defined number of steps (t)
    # NOTE: this takes into account the robot's defined joint angle limits (defined in each robot's config as qlim_min and qlim_max)
    traj: Trajectory() = None
#    if linear:    
#        current = SE3(robot.ets(start=robot.base_link, end=robot.gripper).eval(robot.q))
#        end = SE3(robot.ets(start=robot.base_link, end=robot.gripper).eval(qf))
#        ctraj = rtb.ctraj(T0=current, T1=end, t=frequency)
#        traj = robot.ikine(ctraj)
#    else:
    traj = rtb.mtraj(rtb.trapezoidal, q0=robot.q, qf=qf, t=frequency)

    # Calculate forward kinematic Cartesian trajectory in provided steps (t)
    # cart_traj_normalised = robot.fkine(jtraj.q)

    # Check if trajectory is valid
    if np.max(traj.qd) != 0:
        # Scale the joint trajectories (in steps) to the overall expected move time (sec)
        scaled_qd = traj.qd*(frequency/move_time_sec)
        # print(f"max scaled qd: {np.max(scaled_qd)}")

        # return the generated trajectory scaled with expected speed/time
        return Trajectory(name='trapezoidal-v1', t=move_time_sec, s=traj.q, sd=scaled_qd, sdd=None, istime=True)
    else:
        rospy.logerr(f"Trajectory is invalid --> Cannot Solve.")
        return Trajectory(name='invalid', t=1, s=robot.q, sd=None, sdd=None, istime=False)


def mjtg(robot: rtb.robot, qf: np.ndarray, max_speed: float=0.2, max_rot: float=0.5, frequency=500):
    # This is the average cartesian speed we want the robot to move at
    # NOTE: divided by approx. 2 to make the max speed the approx. peak of the speed achieved
    # TODO: investigate a better approximation strategy here
    rospy.loginfo("Attempting to produce a mjtg tragectory")
    ave_cart_speed = max_speed / 1.92

    #---------------- Calculate Linear move time estimate (3 point sampling)
    # Calculate start and end pose linear distance to estimate the expected time
    current_ee_mat = robot.ets(start=robot.base_link, end=robot.gripper).eval(robot.q)
    mid_ee_mat = robot.ets(start=robot.base_link, end=robot.gripper).eval(qf - (qf - robot.q) / 2)
    end_ee_mat = robot.ets(start=robot.base_link, end=robot.gripper).eval(qf)

    current_ee_pose = current_ee_mat[:3, 3]
    mid_ee_pose = mid_ee_mat[:3, 3]
    end_ee_pose = end_ee_mat[:3, 3]

    # Estimation of time taken based on linear motion from current to end cartesian pose
    # We may require some optimisation of this given the curved nature of the actual ee trajectory
    # NOTE / TODO: thanks Andrew investigate using an arc for a worst case estimate
    D1 = np.sqrt((mid_ee_pose[0] - current_ee_pose[0])**2 +
        (mid_ee_pose[1] - current_ee_pose[1])**2 +
        (mid_ee_pose[2] - current_ee_pose[2])**2)

    D2 = np.sqrt((end_ee_pose[0] - mid_ee_pose[0])**2 +
        (end_ee_pose[1] - mid_ee_pose[1])**2 +
        (end_ee_pose[2] - mid_ee_pose[2])**2)

    linear_move_time = (D1 + D2) / ave_cart_speed
    #---------------- End
    #----------------- Calculate Angular move time estimate (start to end)
    current_ee_rot = current_ee_mat[:3,:3]
    end_ee_rot = end_ee_mat[:3,:3]

    angular_move_time = np.arccos((np.trace(np.transpose(end_ee_rot) @ current_ee_rot) - 1) / 2) / max_rot
    #---------------- End

    move_time = max(linear_move_time, angular_move_time)
    rospy.loginfo(f'Estimated move time of {move_time} (max of) | lin {linear_move_time} | ang {angular_move_time}')
    # Edited as part of branch hotfix/96fd293: termination on invalid trajectory
    if move_time == 0:
        rospy.logerr(f"Trajectory is invalid --> Cannot Solve.")
        return Trajectory(name='invalid', t=1, s=robot.q, sd=None, sdd=None, istime=False)

    # Obtain minimum jerk velocity profile of joints based on estimated end effector move time
    q = []
    qd = []
    # TODO: investigate if this is a useful parameter to provide when running the trajectory
    qdd = []

    # Calculate time frequency - based on the max time required for trajectory and the frequency of operation
    # TODO: compute the max delta in joint space vs the max 'open loop' control delta suitable for the arm...
    # something like -- max_joint_delta / max_control_joint_delta
    # telling us for the largest joint movement how many steps we would require to remain accurate...
    # then make sure we use this as our 'frequency' in the below 'timefreq' calculation
    # *** this means our number of steps scales with max trajectory velocity rather than a fixed control frequency
    # NOTE: Impementation taken from
    # -- PAPER: https://arxiv.org/ftp/arxiv/papers/2102/2102.07459.pdf#:~:text=The%20minimum%20jerk%20trajectory%20as,is%20the%20free-hand%20motion.
    # -- PYTHON: https://mika-s.github.io/python/control-theory/trajectory-generation/2017/12/06/trajectory-generation-with-a-minimum-jerk-trajectory.html
    # NOTE: !!! When time permits this implementation should be replaced or at least benchmarked against
    # -- RoboticsToolBox: https://www.mathworks.com/help/robotics/ug/plan-minimum-jerk-trajectory-for-robot-arm.html
    # -- RoboticsToolBox: https://www.mathworks.com/help/robotics/ref/minjerkpolytraj.html
    timefreq = int(move_time * frequency)
    for time in range(1, timefreq):
        q.append(
            robot.q + (qf - robot.q) *
            (10.0 * (time/timefreq)**3
            - 15.0 * (time/timefreq)**4
            + 6.0 * (time/timefreq)**5))

        qd.append(
            frequency * (1.0/timefreq) * (qf - robot.q) *
            (30.0 * (time/timefreq)**2.0
            - 60.0 * (time/timefreq)**3.0
            + 30.0 * (time/timefreq)**4.0))
    
    rospy.loginfo(f"RETURNING a mjtg tragectory - len q {len(q)}, len qd {len(qd)}")
    return Trajectory(name='minimum-jerk', t=move_time, s=q, sd=qd, sdd=None, istime=True)

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
