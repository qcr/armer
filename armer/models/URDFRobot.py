#!/usr/bin/env python3
"""
URDFRobot module defines the URDFRobot type

.. codeauthor:: Gavin Suddreys
.. codeauthor:: Dasun Gunasinghe
"""

import numpy as np
import re
import time
import rospkg
from io import BytesIO
from roboticstoolbox.robot import Robot, Link, ET, ETS
from roboticstoolbox.tools import URDF
import xml.etree.ElementTree as ETT
import spatialmath

class URDFRobot(Robot):
  def __init__(self,
               nh,
               qz=None,
               qr=None,
               gripper=None,
               tool=None,
               urdf_file=None,
               *args,
               **kwargs):
    self.nh = nh

    if urdf_file:
      links, name, urdf_string, urdf_filepath = self.URDF_read(urdf_file)
    else:
      links, name, urdf_string, urdf_filepath = self.URDF_read_description()
    
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
    self.logger('[INIT] Waiting for robot description')
    while not self.nh.has_param('/robot_description'):
      time.sleep(0.5)
    self.logger('[INIT] Found robot description')

    urdf_string = self.URDF_resolve(self.get_parameter('/robot_description'))
    
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

  @staticmethod
  def resolve_ets(tool):
    transforms = [ET.tx(0).A()]
    
    if 'ets' in tool:
      transforms = [ getattr(ET, list(et.keys())[0])(list(et.values())[0]) for et in tool['ets'] ]

    return ETS(transforms)
  
  def get_parameter(self, param_name):
    return self.nh.get_parameter(param_name) if hasattr(self.nh, 'get_parameter') else self.nh.get_param(param_name)
  
  def logger(self, message, mode='info'):
    if hasattr(self.nh, 'get_logger'):
      if mode == 'error':
        self.nh.get_logger().error(message)
      elif mode == 'warn':
        self.nh.get_logger().warn(message)
      else:
        self.nh.get_logger().info(message)
    elif hasattr(self.nh, 'loginfo'):
      if mode == 'error':
        self.nh.logerror(message)
      elif mode == 'warn':
        self.nh.logwarn(message)
      else:
        self.nh.loginfo(message)
    else:
      print(message)

if __name__ == "__main__":  # pragma nocover

    # TODO: change this to a known, more tracable xacro
    r = URDFRobot(urdf_file='ur_description/urdf/ur5_joint_limited_robot.urdf.xacro')
    print(r)
    