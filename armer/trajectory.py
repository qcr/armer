import numpy as np
import rospy
import scipy
import roboticstoolbox as rtb
from roboticstoolbox.tools.trajectory import Trajectory

class TrajectoryExecutor:
  def __init__(self, robot: rtb.ERobot, traj: Trajectory):

    self.robot: rtb.ERobot = robot
    self.traj: Trajectory = traj
    
    self.last_jp = np.zeros(self.robot.n)

    self.time_step = 0
    
    self.cartesian_ee_vel_vect = [] # logging

    self.is_aborted = False

    self._finished = False
    self._success = False
    

    if self.traj.istime and len(self.traj.s) >= 2:
      s = np.linspace(0, 1, len(self.traj.s))
      self.qfunc = scipy.interpolate.interp1d(s, np.array(self.traj.s), axis=0)
      self.qdfunc = scipy.interpolate.interp1d(s, np.array(self.traj.sd), axis=0)

  def step(self, dt: float):
    # Self termination if within goal space
    if self.is_finished(cutoff=0.01):
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
      req_jp = self.qfunc(self.time_step / self.traj.t)
      req_jv = self.qdfunc(self.time_step / self.traj.t)
    else:
      req_jp = self.traj.s[self.time_step]
      req_jv = self.traj.sd[self.time_step]

    # Calculate error in joint velocities based on current and expected
    current_jv = current_jp - self.last_jp
    erro_jv = req_jv - current_jv
    erro_jp = req_jp - current_jp

    self.last_jp = np.array(current_jp)
    
    # Calculate corrected error based on error above
    corr_jv = current_jv + (erro_jv * self.robot.Kp) + (erro_jp * self.robot.Ki) + (((req_jv - current_jv) / self.robot.frequency) * self.robot.Kd)
    
    # corr_jv = np.zeros(self.robot.n)
    if np.any(np.max(np.fabs(erro_jp)) > 0.5):
        rospy.logerr('Exceeded delta joint position max')
        self._finished = True
        
    # Increment time step(s)
    self.time_step += dt if self.traj.istime else 1
    return corr_jv

  def is_finished(self, cutoff=0.01):
    if self._finished:
      return True

    if len(self.traj.s) < 2 or np.all(np.fabs(self.traj.s[-1] - self.robot.q) < cutoff):
      rospy.loginfo('Too close to goal, quitting movement {}...'.format(self.time_step / self.traj.t))
      self._finished = True
      self._success = True
    
    if (self.time_step) >= self.traj.t - (1 if not self.traj.istime else 0):
      
      rospy.loginfo('Timed out')
      print(np.fabs(self.traj.s[-1] - self.robot.q))
      self._finished = True
      self._success = True
      
    return self._finished

  def is_succeeded(self):
    return self._success
