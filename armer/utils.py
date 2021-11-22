"""
Utility functions used by Armer

.. codeauthor:: Gavin Suddreys
"""

import rospy
import numpy as np
import math
from spatialmath import SE3, SO3, UnitQuaternion, base
from geometry_msgs.msg import TransformStamped
import qpsolvers as qp
import copy
import roboticstoolbox as rtb
import random
from trac_ik_python.trac_ik import IK


# # IK and QPJ Solver code provided by Jesse Haviland
# def rand_q(n, qlim):
#     q = np.zeros(n)
#     for i in range(n):
#         q[i] = random.uniform(qlim[0, i], qlim[1, i])
#     return q

# def gen_pose(robot, qlim):
#     q = rand_q(robot.n, qlim)
#     Tep = robot.fkine(q)
#     return Tep, q

# def error(Te, Tep, threshold):
#     eTep = np.linalg.inv(Te) @ Tep
#     e = np.empty(6)

#     # Translational error
#     e[:3] = eTep[:3, -1]

#     # Angular error
#     e[3:] = base.tr2rpy(eTep, unit="rad", order="zyx", check=False)
#     print(e)
#     return e, True if np.sum(np.abs(e)) < threshold else False

# def angle_axis(T, Td):
#     e = np.zeros(6)

#     # e[:3] = Td.t - T.t
#     e[:3] = Td[:3, -1] - T[:3, -1]

#     # R = Td.R @ T.R.T
#     R = Td[:3, :3] @ T[:3, :3].T

#     li = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

#     if base.iszerovec(li):
#         # diagonal matrix case
#         if np.trace(R) > 0:
#             # (1,1,1) case
#             a = np.zeros((3,))
#         else:
#             a = np.pi / 2 * (np.diag(R) + 1)
#     else:
#         # non-diagonal matrix case
#         ln = base.norm(li)
#         a = math.atan2(ln, np.trace(R) - 1) * li / ln

#     e[3:] = a

#     return e


# class IK:
#     def __init__(self, name, attempts, it_max, problems):
#         self.name = name
#         self.attempts = attempts
#         self.it_max = it_max
#         self.results = np.empty(problems)
#         self.t_attempts = np.zeros(problems)
#         self.limit = np.zeros(problems)


# class QP(IK):
#     def __init__(self, robot, q0, Tep, end=None, threshold=1e-10, name="QP-J-Limits", attempts=50, it_max=500, problems=1000):
#         super().__init__(name, attempts, it_max, problems)
#         self.q = q0
#         self.robot = robot
#         self.Tep = Tep
#         self.end = end
#         self.threshold = threshold

#     def step(self):
#         Te = self.robot.fkine(self.q, end=self.end, fast=True)

#         e, arrived = error(Te, self.Tep, self.threshold)
        
#         if arrived:
#             return True

#         J = self.robot.jacob0(self.q, end=self.end, fast=True)

#         # Gain term (lambda) for control minimisation
#         Y = 0.01

#         # Quadratic component of objective function
#         Q = np.eye(self.robot.n + 6)

#         # Joint velocity component of Q
#         Q[: self.robot.n, : self.robot.n] *= Y

#         # Slack component of Q
#         Q[self.robot.n :, self.robot.n :] = (1 / np.sum(np.abs(e))) * np.eye(6)

#         # The equality contraints
#         Aeq = np.c_[J, np.eye(6)]
#         beq = e.reshape((6,))

#         # The inequality constraints for joint limit avoidance
#         Ain = np.zeros((self.robot.n + 6, self.robot.n + 6))
#         bin = np.zeros(self.robot.n + 6)

#         # The minimum angle (in radians) in which the joint is allowed to approach
#         # to its limit
#         ps = 0.05

#         # The influence angle (in radians) in which the velocity damper
#         # becomes active
#         pi = 0.9

#         # Form the joint limit velocity damper
#         Ain[: self.robot.n, : self.robot.n], bin[: self.robot.n] = self.robot.joint_velocity_damper(
#             ps, pi, self.robot.n
#         )

#         c = np.zeros(self.robot.n + 6)
#         # c = np.r_[-robot.jacobm().reshape((robot.n,)), np.zeros(6)]

#         qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=None, ub=None)

#         self.q += qd[: self.robot.n]

#         return False

def ikine(robot, target, q0, end, threshold=2, attempts=10):
    bx = by = bz = 0.001
    brx = bry = brz = 0.1
    
    ik_solver = IK(robot.base_link.name,
                   end,
                   timeout=0.1,
                   solve_type='Manipulation1')
    
    ik_solver.set_joint_limits(*robot.qlim)

    sol = ik_solver.get_ik(q0,
                        target.position.x, target.position.y, target.position.z,
                        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w,
                        bx, by, bz,
                        brx, bry, brz)

    return type('obj', (object,), {'q' : sol})
    


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
