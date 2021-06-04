#!/usr/bin/env python3
"""
Armer Class

.. codeauthor:: Gavin Suddreys
"""
from __future__ import annotations

from typing import List
import timeit
import importlib

import rospy
import tf2_ros
import yaml

import roboticstoolbox as rtb
from roboticstoolbox.backends.Swift import Swift

from spatialmath.base.argcheck import getvector

from armer.utils import populate_transform_stamped

from armer.robots import ROSRobot
from armer.timer import Timer


class Armer:
    """
    The Armer Driver.

    :param robot: [description], List of robots to be managed by the driver
    :type robots: List[rtb.robot.Robot], optional
    :param backend: [description], defaults to None
    :type backend: rtb.backends.Connector, optional

    .. codeauthor:: Gavin Suddrey
    .. sectionauthor:: Gavin Suddrey
    """

    # pylint: disable=too-many-instance-attributes

    def __init__(
            self,
            robots: List[rtb.robot.Robot] = None,
            backend: rtb.backends.Connector = None,
            publish_transforms: bool = False) -> None:

        self.robots: List[ROSRobot] = robots
        self.backend: rtb.backends.Connector = backend
        self.read_only_backends : List[rtb.backends.Connector] = []

        if not self.robots:
            self.robots = [ROSRobot(self, rtb.models.URDF.UR5())]

        if not self.backend:
            self.backend = Swift()

        self.is_publishing_transforms = publish_transforms

        self.broadcaster: tf2_ros.TransformBroadcaster = None

        if self.is_publishing_transforms:
            self.broadcaster = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(500)
        self.last_tick = timeit.default_timer()

        # Launch backend
        self.backend.launch()

        for robot in self.robots:
            self.backend.add(robot)

        for readonly in self.read_only_backends:
            readonly.launch()

            for robot in self.robots:
                readonly.add(robot, readonly=True)


    def close(self):
        """
        Close backend and stop action servers
        """
        self.backend.close()

        for robot in self.robots:
            robot.close()

    def publish_transforms(self) -> None:
        """[summary]
        """
        if not self.is_publishing_transforms:
            return

        for robot in self.robots:
            joint_positions = getvector(robot.q, robot.n)

            for link in robot.elinks:
                if link.parent is None:
                    continue

                if link.isjoint:
                    transform = link.A(joint_positions[link.jindex])
                else:
                    transform = link.A()

                self.broadcaster.sendTransform(populate_transform_stamped(
                    link.parent.name,
                    link.name,
                    transform
                ))

            for gripper in robot.grippers:
                joint_positions = getvector(gripper.q, gripper.n)

                for link in gripper.links:
                    if link.parent is None:
                        continue

                    if link.isjoint:
                        transform = link.A(joint_positions[link.jindex])
                    else:
                        transform = link.A()

                    self.broadcaster.sendTransform(populate_transform_stamped(
                        link.parent.name,
                        link.name,
                        transform
                    ))

    @staticmethod
    def load(path: str) -> Armer:
        """
        Generates an Armer Driver instance from the configuration file at path

        :param path: The path to the configuration file
        :type path: str
        :return: An Armer driver instance
        :rtype: Armer
        """
        with open(path, 'r') as handle:
            config = yaml.load(handle, Loader=yaml.SafeLoader)

        robots: List[rtb.robot.Robot] = []

        for spec in config['robots']:
            module_name, model_name = spec['model'].rsplit('.', maxsplit=1)
            robot_cls = getattr(importlib.import_module(module_name), model_name)
            del spec['model']

            wrapper = ROSRobot

            if 'type' in spec:
                module_name, model_name = spec['type'].rsplit('.', maxsplit=1)
                wrapper = getattr(importlib.import_module(module_name), model_name)
                del spec['type']

            robots.append(wrapper(robot_cls(), **spec))

        return Armer(robots=robots)

    def run(self) -> None:
        """
        Runs the driver. This is a blocking call.
        """
        self.last_tick = timeit.default_timer()

        while not rospy.is_shutdown():
            with Timer('ROS', True):
                current_time = timeit.default_timer()
                dt = current_time - self.last_tick

                for robot in self.robots:
                    robot.step(dt=dt)

                self.backend.step(dt=dt)

                for backend in self.read_only_backends:
                    backend.step(dt=dt)

                self.publish_transforms()

                self.rate.sleep()

                self.last_tick = current_time


if __name__ == '__main__':
    rospy.init_node('manipulator')
    manipulator = Armer(publish_transforms=False)
    manipulator.run()
