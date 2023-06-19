#!/usr/bin/env python
"""
URDFRobot module defines the URDFRobot type

.. codeauthor:: Gavin Suddreys
.. codeauthor:: Dasun Gunasinghe
"""

import numpy as np
import re
import rospy
import rospkg
from io import BytesIO
from roboticstoolbox.robot import Robot, Link, ET, ETS
from roboticstoolbox.tools import URDF
import xml.etree.ElementTree as ETT
import spatialmath

class URDFRobot(Robot):
  def __init__(self,
               qz=None,
               qr=None,
               gripper=None,
               tool=None,
               urdf_file=None,
               *args,
               **kwargs):

    if urdf_file:
      links, name, urdf_string, urdf_filepath = self.URDF_read(urdf_file)
    else:
      links, name, urdf_string, urdf_filepath = URDFRobot.URDF_read_description()
    
    self.gripper = gripper if gripper else URDFRobot.resolve_gripper(links)
    gripper_link = list(filter(lambda link: link.name == self.gripper, links))
    
    if tool:
      ets = URDFRobot.resolve_ets(tool)
      
      if 'name' in tool:
        links.append(Link(ets, name=tool['name'], parent=self.gripper))
        self.gripper = tool['name']

      gripper_link[0].tool = spatialmath.SE3(ets.compile()[0].A())
      
    # DEBUGGING
    # print(f"links:")
    # for link in links:
    #   print(link)
    # print(f"gripper: {self.gripper} | gripper link: {gripper_link}")

    super().__init__(
        links,
        name=name,
        gripper_links=gripper_link,
        urdf_string=urdf_string,
        urdf_filepath=urdf_filepath,
    )

    self.qr = qr if qr else np.array([0] * self.n)
    self.qz = qz if qz else np.array([0] * self.n)

    self.addconfiguration("qr", self.qr)
    self.addconfiguration("qz", self.qz)
  
  @staticmethod
  def URDF_resolve(urdf_string):
    rospack = rospkg.RosPack()
    packages = list(set(re.findall(r'(package:\/\/([^\/]*))', urdf_string)))
    
    for package in packages:
      urdf_string = urdf_string.replace(package[0], rospack.get_path(package[1]))
    
    return urdf_string
  
  @staticmethod
  def URDF_read_description(wait=True):
    if wait:
      rospy.loginfo('[INIT] Waiting for robot description')
      while not rospy.has_param('/robot_description'):
        rospy.sleep(0.5)
      rospy.loginfo('[INIT] Found robot description')
    else:
      # Check if robot param exists and handle as error if need be
      param = rospy.has_param('/robot_description')
      if not param: return None, None, None, None
      rospy.loginfo(f"[INIT] Found robot description NEW")

    urdf_string = URDFRobot.URDF_resolve(rospy.get_param('/robot_description'))
    
    tree = ETT.parse(
      BytesIO(bytes(urdf_string, "utf-8")), 
      parser=ETT.XMLParser()
    )

    node = tree.getroot()
    urdf = URDF._from_xml(node, '/')

    return urdf.elinks, urdf.name, urdf_string, '/'

  @staticmethod
  def resolve_gripper(links):
    parents = []
    
    for link in links:
      if not link.parent:
        continue

      if link.parent.name in parents:
        return link.parent.name
      
      parents.append(link.parent.name)
    
    return links[-1].name

  @staticmethod
  def resolve_ets(tool):
    transforms = [ET.tx(0).A()]
    
    if 'ets' in tool:
      transforms = [ getattr(ET, list(et.keys())[0])(list(et.values())[0]) for et in tool['ets'] ]

    return ETS(transforms)

if __name__ == "__main__":  # pragma nocover

    r = URDFRobot(urdf_file='ur_description/urdf/ur5_joint_limited_robot.urdf.xacro')
    print(r)
    