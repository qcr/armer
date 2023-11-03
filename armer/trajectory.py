"""
Trajectory Executor class used by Armer

.. codeauthor:: Gavin Suddrey
.. codeauthor:: Dasun Gunasinghe
"""

import numpy as np
import rospy
import scipy
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import Trajectory

class TrajectoryExecutor:
  def __init__(self, robot: rtb.Robot, traj: Trajectory, cutoff: float=0.01):

    self.robot: rtb.Robot = robot
    self.traj: Trajectory = traj
    self.traj_max_time = np.max(self.traj.t)
    
    self.last_jp = np.zeros(self.robot.n)

    self.time_step = 0
    
    self.cartesian_ee_vel_vect = [] # logging

    self.is_aborted = False

    self._finished = False
    self._success = False

    self.cutoff = cutoff

    #DEBUGGING
    print(f"max traj time: {self.traj_max_time}")

    if self.traj.istime and len(self.traj.s) >= 2:
      s = np.linspace(0, 1, len(self.traj.s))
      self.qfunc = scipy.interpolate.interp1d(s, np.array(self.traj.s), axis=0)
      self.qdfunc = scipy.interpolate.interp1d(s, np.array(self.traj.sd), axis=0)

  def step(self, dt: float):
    # Self termination if within goal space
    if self._finished or self.is_finished():
      return np.zeros(self.robot.n)

    # Compute current state jacobian
    jacob0 = self.robot.jacob0(self.robot.q, end=self.robot.gripper)

    # Get current joint velocity and calculate current twist
    current_jv = self.robot.state.joint_velocities #elf.j_v
    current_jp = self.robot.state.joint_poses

    current_twist = jacob0 @ current_jv
    current_linear_vel = np.linalg.norm(current_twist[:3])
    self.cartesian_ee_vel_vect.append(current_linear_vel)

    # Calculate required joint velocity at this point in time based on trajectory
    if self.traj.istime:
      req_jp = self.qfunc(self.time_step / self.traj_max_time)
      req_jv = self.qdfunc(self.time_step / self.traj_max_time)
    else:
      req_jp = self.traj.s[self.time_step]
      req_jv = self.traj.sd[self.time_step]

    # Calculate error in joint velocities based on current and expected
    erro_jv = req_jv - current_jv
    erro_jp = req_jp - current_jp

    self.last_jp = np.array(current_jp)
    
    # Calculate corrected error based on error above
    corr_jv = current_jv + erro_jv
    
    # TODO: remove debugging - note that this method is sometimes preempted...
    # DEBUG
    # print(f"---")
    # print(f"time step is: {self.time_step}")
    # print(f"current jv is: {current_jv} | alt (self.j_v) is: {self.robot.j_v}")
    # print(f"exp jv is: {req_jv}")
    # print(f"error in jv is: {erro_jv}")
    # print(f"corrected jv is: {corr_jv}")
    # print(f"###")

    # TODO: Validate 
    # - has this max delta been checked on xArm and others?
    # - should it be exposed as a parameter?
    if np.any(np.max(np.fabs(erro_jp)) > 0.5):
        rospy.logerr('Exceeded delta joint position max')
        self._finished = True
        
    # Increment time step(s)
    self.time_step += dt if self.traj.istime else 1

    return corr_jv

  def abort(self):
    self._finished = True
    self._success = False

  def is_finished(self, cutoff=None):
    if self._finished:
      return True
    
    # Initial calls use default until a different cutoff is provided
    if cutoff != None:
      self.cutoff = cutoff
    else:
      cutoff = self.cutoff

    if len(self.traj.s) < 2 or np.all(np.fabs(self.traj.s[-1] - self.robot.q) < cutoff):
      if len(self.traj.s) > 0:
        rospy.loginfo("Trajectory length is NOT the problem")
        link_errors = np.fabs(self.traj.s[-1] - self.robot.q)
        rospy.loginfo(f'Link errors used to apply cutoff ({cutoff}): \n{link_errors}')
      rospy.loginfo(f'Too close to goal {(self.time_step / self.traj_max_time)}')
      if self.cartesian_ee_vel_vect:
        rospy.loginfo(f"Max cartesian speed: {np.max(self.cartesian_ee_vel_vect)}")
      self._finished = True
      self._success = True
    
    if (self.time_step) >= self.traj_max_time - (1 if not self.traj.istime else 0):
      rospy.loginfo(f'Timed out | End time: {self.time_step}')
      if self.cartesian_ee_vel_vect:
        rospy.loginfo(f"Max cartesian speed: {np.max(self.cartesian_ee_vel_vect)}")
      self._finished = True
      self._success = True
      
    return self._finished

  def is_succeeded(self):
    return self._success
