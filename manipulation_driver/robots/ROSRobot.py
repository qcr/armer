import copy

import rospy
import roboticstoolbox as rtb
import spatialmath as sp
import numpy as np
from typing import Any

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from rv_msgs.msg import JointVelocity, ManipulatorState

class ROSRobot(rtb.ERobot):
  def __init__(self, 
    robot: rtb.robot.Robot, 
    joint_state_topic: str = None, 
    joint_velocity_topic: str = None, 
    *args, 
    **kwargs):

    self.__dict__.update(robot.__dict__)
    
    self.joint_names = list(map(lambda link: link.name.replace('link', 'joint'), filter(lambda link: link.isjoint, self.elinks)))
    self.joint_indexes = []
    
    self.qm = robot.q
    self.tau = [0] * len(robot.q)

    self.joint_subscriber = rospy.Subscriber(
        joint_state_topic if joint_state_topic else '/joint_states', 
        JointState, 
        self._state_cb
    )
    self.joint_publisher = rospy.Publisher(
      joint_velocity_topic if joint_velocity_topic else '/joint_velocity_node_controller/joint_velocity', 
      JointVelocity, 
      queue_size=1
    )
    
  def step(self, dt=0.01):
    self.q = self.joint_states
    self.joint_publisher.publish(JointVelocity(joints=self.qd))

  def recover(self):
    rospy.logwarn('Recovery not implemented for this arm')

  def state(self):
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

  def set_cartesian_impedance(self, values):
    rospy.logwarn('Setting cartesian impedance not implemented for this arm')
    return True

  def _state_cb(self, msg):
    if not self.joint_indexes:
      self.joint_indexes = [idx for idx, joint_name in enumerate(msg.name) if joint_name in self.joint_names]
    
    self.qm = np.array(msg.position)[self.joint_indexes]
    self.tau = np.array(msg.effort)[self.joint_indexes]

