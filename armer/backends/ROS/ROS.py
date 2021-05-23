#!/usr/bin/env python
"""
@author Gavin Suddrey
"""

import time
import subprocess
import os

from roboticstoolbox.backends.Connector import Connector

import armer

class ROS(Connector):  # pragma nocover
    """
    ROS Backend
    """

    def __init__(self):
        super().__init__()

        self.roscore = None
        self.robots = {}

    #
    #  Basic methods to do with the state of the external program
    #

    def launch(self, roscore=False, ros_master_uri=None, ros_ip=None): # pylint: disable=arguments-differ
        """
        Launch the backend and establish any ROS connections
        """

        super().launch()

        # Launch roscore in seperate process
        if roscore:
            print('Launching ROS core\n')
            self.roscore = subprocess.Popen('roscore')

            # Give it some time to launch
            time.sleep(1)

        if ros_master_uri:
            os.environ["ROS_MASTER_URI"] = ros_master_uri

        if ros_ip:
            os.environ["ROS_IP"] = ros_ip

    #
    #  Methods to interface with the robots created in other environemnts
    #

    def add(self, ob): # pylint: disable=arguments-differ
        """
        Add a robot to the environment
        """
        if isinstance(ob, armer.robots.ROSRobot):
            self.robots[ob.name] = ob

        super().add()
