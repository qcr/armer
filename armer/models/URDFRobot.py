#!/usr/bin/env python

import numpy as np
import re
import rospy
import rospkg
from io import BytesIO
from roboticstoolbox.robot.ERobot import ERobot
from roboticstoolbox.tools import URDF
import xml.etree.ElementTree as ETT

class URDFRobot(ERobot):
  def __init__(self,
               qz=None,
               qr=None,
               gripper=None,
               urdf_file=None,
               *args,
               **kwargs):

    if urdf_file:
      links, name, urdf_string, urdf_filepath = self.URDF_read(urdf_file)
    else:
      links, name, urdf_string, urdf_filepath = self.URDF_read_description()
    
    self.gripper = gripper if gripper else URDFRobot.resolve_gripper(links)
    gripper_link = list(filter(lambda link: link.name == self.gripper, links))
    
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

  def URDF_read_description(self):
    rospy.loginfo('Waiting for robot description')
    while not rospy.has_param('/robot_description'):
      rospy.sleep(0.5)
    rospy.loginfo('Found robot description')

    urdf_string = self.URDF_resolve(rospy.get_param('/robot_description'))
    
    tree = ETT.parse(
      BytesIO(bytes(urdf_string, "utf-8")), 
      parser=ETT.XMLParser()
    )

    node = tree.getroot()
    urdf = URDF._from_xml(node, '/')

    return urdf.elinks, urdf.name, urdf_string, '/'
    
  def URDF_resolve(self, urdf_string):
    rospack = rospkg.RosPack()
    packages = list(set(re.findall(r'(package:\/\/([^\/]*))', urdf_string)))
    
    for package in packages:
      urdf_string = urdf_string.replace(package[0], rospack.get_path(package[1]))
    
    return urdf_string

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

if __name__ == "__main__":  # pragma nocover

    r = URDFRobot(urdf_file='ur_description/urdf/ur5_joint_limited_robot.urdf.xacro')
    print(r)
    