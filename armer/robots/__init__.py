"""
Robots for Armer

.. codeauthor:: Gavin Suddreys
"""
import os

print('ROS Version: {}'.format(os.environ['ROS_VERSION']))

from armer.robots.BaseRobot import BaseRobot

if os.environ['ROS_VERSION'] == '1':
  from armer.robots.ROS1Robot import ROS1Robot as ROSRobot
else:
  from armer.robots.ROS2Robot import ROS2Robot as ROSRobot

__all__ = [
    'BaseRobot',
    'ROSRobot'
]
