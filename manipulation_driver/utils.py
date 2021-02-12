"""
Utility functions used by the Manipulation Driver

.. codeauthor:: Gavin Suddreys
"""

import rospy
from spatialmath import SE3, UnitQuaternion
from geometry_msgs.msg import TransformStamped

def populate_transform_stamped(parent_name: str, link_name: str, transform: SE3):
    """
    Generates a geometry_msgs/TransformStamped message between
    a link and its parent based on the provided SE3 transform
    """
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = parent_name
    transform_stamped.child_frame_id = link_name
    transform_stamped.transform.translation.x = transform.t[0]
    transform_stamped.transform.translation.y = transform.t[1]
    transform_stamped.transform.translation.z = transform.t[2]

    rot = UnitQuaternion(transform.R)

    transform_stamped.transform.rotation.w = rot.A[0]
    transform_stamped.transform.rotation.x = rot.A[1]
    transform_stamped.transform.rotation.y = rot.A[2]
    transform_stamped.transform.rotation.z = rot.A[3]

    return transform_stamped
