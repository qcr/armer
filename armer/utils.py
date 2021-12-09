"""
Utility functions used by Armer

.. codeauthor:: Gavin Suddrey
"""

import rospy
import numpy as np
from spatialmath import SE3, SO3, UnitQuaternion, base
from geometry_msgs.msg import TransformStamped
import qpsolvers as qp
import roboticstoolbox as rtb
from trac_ik_python.trac_ik import IK

def ikine(robot, target, q0, end):
    bx = by = bz = 0.001
    brx = bry = brz = 0.01
    
    ik_solver = IK(robot.base_link.name,
                   end,
                   timeout=0.1,
                   urdf_string=robot.urdf_string,
                   solve_type='Manipulation1')
    
    ik_solver.set_joint_limits(*robot.qlim)
    
    sol = ik_solver.get_ik(q0,
                        target.position.x, target.position.y, target.position.z,
                        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w,
                        bx, by, bz,
                        brx, bry, brz)

    return type('obj', (object,), {'q' : np.array(sol)})
    


    # Temp addition of minimum jerk trajectory calculator
    # Reference: https://mika-s.github.io/python/control-theory/trajectory-generation/2017/12/06/trajectory-generation-with-a-minimum-jerk-trajectory.html
def mjtg(current, setpoint, frequency, move_time):
    q = []
    qd = []
    
    timefreq = int(move_time * frequency)

    for time in range(1, timefreq):
        q.append(
            current + (setpoint - current) *
            (10.0 * (time/timefreq)**3
            - 15.0 * (time/timefreq)**4
            + 6.0 * (time/timefreq)**5))

        qd.append(
            frequency * (1.0/timefreq) * (setpoint - current) *
            (30.0 * (time/timefreq)**2.0
            - 60.0 * (time/timefreq)**3.0
            + 30.0 * (time/timefreq)**4.0))

    return q, qd

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
