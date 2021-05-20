"""
ROSRobot module defines the ROSRobot type

.. codeauthor:: Gavin Suddreys
"""
from typing import List

import rospy
import roboticstoolbox as rtb
import spatialmath as sp
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from armer_msgs.msg import JointVelocity, ManipulatorState

# pylint: disable=too-many-instance-attributes

class ROSRobot(rtb.ERobot):
    """
    The ROSRobot class wraps the rtb.ERobot implementing basic ROS functionality
    """
    def __init__(self,
                 robot: rtb.robot.Robot,
                 joint_state_topic: str = None,
                 joint_velocity_topic: str = None,
                 *args,
                 **kwargs):  #pylint: disable=unused-argument

        super().__init__(robot)

        self.__dict__.update(robot.__dict__)

        self.joint_names = list(map(lambda link: link.name.replace(
            'link', 'joint'), filter(lambda link: link.isjoint, self.elinks)))
        self.joint_indexes = []

        self.joints_measured = robot.q
        self.tau = [0] * len(robot.q)

        self.joint_subscriber = rospy.Subscriber(
            joint_state_topic if joint_state_topic else '/joint_states',
            JointState,
            self._state_cb
        )
        self.joint_publisher = rospy.Publisher(
            joint_velocity_topic
                if joint_velocity_topic
                else '/joint_velocity_node_controller/joint_velocity',
            JointVelocity,
            queue_size=1
        )

    def step(self, dt: float=0.01) -> None: #pylint: disable=unused-argument
        """
        Updates the robot joints (robot.q) used in computing kinematics
        :param dt: the delta time since the last update, defaults to 0.01
        :type dt: float, optional
        """
        self.q = self.joints_measured
        self.joint_publisher.publish(JointVelocity(joints=self.qd))

    def state(self) -> ManipulatorState:
        """
        Generates a ManipulatorState message for the robot

        :return: ManipulatorState message describing the current state of the robot
        :rtype: ManipulatorState
        """
        ee_pose = self.fkine(self.q)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'panda_link0'

        pose_stamped.pose.position.x = ee_pose.t[0]
        pose_stamped.pose.position.y = ee_pose.t[1]
        pose_stamped.pose.position.z = ee_pose.t[2]

        ee_rot = sp.UnitQuaternion(ee_pose.R)

        pose_stamped.pose.orientation.w = ee_rot.A[0]
        pose_stamped.pose.orientation.x = ee_rot.A[1]
        pose_stamped.pose.orientation.y = ee_rot.A[2]
        pose_stamped.pose.orientation.z = ee_rot.A[3]

        state = ManipulatorState()
        state.ee_pose = pose_stamped
        state.joint_poses = list(self.q)
        state.joint_torques = list(self.tau)

        return state

    def recover(self) -> bool: #pylint: disable=no-self-use
        """
        Robot specific method to recover from any errors

        Note: This method should be overloaded for robots that support this functionality

        :return: True if error recovery was successful
        :rtype: bool
        """
        rospy.logwarn('Recovery not implemented for this arm')
        return True

    def set_cartesian_impedance(self, values: List[float]) -> bool: #pylint: disable=no-self-use,unused-argument
        """
        Robot specific method to set 6-DOF cartesian impedance

        Note: This method should be overloaded for robots that support this functionality

        :param values: The impedance values for each axis (xyzrpy)
        :type values: List[float]
        :return: True if the values on the robot were updated successfully
        :rtype: bool
        """
        rospy.logwarn('Setting cartesian impedance not implemented for this arm')
        return True

    def _state_cb(self, msg):
        if not self.joint_indexes:
            self.joint_indexes = [idx for idx, joint_name in enumerate(
                msg.name) if joint_name in self.joint_names]

        self.joints_measured = np.array(msg.position)[self.joint_indexes]
        self.tau = np.array(msg.effort)[self.joint_indexes]
