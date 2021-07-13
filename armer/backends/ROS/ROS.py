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

    def step(self, dt=0.01):
        '''
        step(dt) triggers the external program to make a time step
        of defined time updating the state of the environment as defined by
        the robot's actions.

        The will go through each robot in the list and make them act based on
        their control type (position, velocity, acceleration, or torque). Upon
        acting, the other three of the four control types will be updated in
        the internal state of the robot object. The control type is defined
        by the robot object, and not all robot objects support all control
        types.

        '''

        pass

    def reset(self):
        '''
        reset() triggers the external program to reset to the
        original state defined by launch

        '''

        pass

    def restart(self):
        '''
        restart() triggers the external program to close and relaunch
        to thestate defined by launch

        '''

        pass

    def close(self):
        '''
        close() triggers the external program to gracefully close

        '''

        pass

    #
    #  Methods to interface with the robots created in other environemnts
    #

    def remove(self):
        '''
        remove(id) removes the object from the external environment.

        '''

        pass

    def hold(self):    # pragma nocover
        '''
        hold() keeps the backend open i.e. stops the program from closing once
        the main script has finished. This method may need keep an even loop
        running for the backend to keep it responsive.

        '''

        pass