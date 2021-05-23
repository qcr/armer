#!/usr/bin/env python3
"""
Armer Class

.. codeauthor:: Gavin Suddreys
"""
import os


from typing import List, Any
import timeit

import rospy
import tf2_ros
import yaml

import roboticstoolbox as rtb
from roboticstoolbox.backends.Swift import Swift

from spatialmath.base.argcheck import getvector

from armer.utils import populate_transform_stamped

from armer.robots import ROSRobot
from armer.timer import Timer

from armer_msgs.srv import AddNamedPose, \
    AddNamedPoseRequest, \
    AddNamedPoseResponse

from armer_msgs.srv import AddNamedPoseConfig, \
    AddNamedPoseConfigRequest, \
    AddNamedPoseConfigResponse

from armer_msgs.srv import GetNamedPoseConfigs, \
    GetNamedPoseConfigsRequest, \
    GetNamedPoseConfigsResponse

from armer_msgs.srv import GetNamedPoses, \
    GetNamedPosesRequest, \
    GetNamedPosesResponse

from armer_msgs.srv import RemoveNamedPose, \
    RemoveNamedPoseRequest, \
    RemoveNamedPoseResponse

from armer_msgs.srv import RemoveNamedPoseConfig, \
    RemoveNamedPoseConfigRequest, \
    RemoveNamedPoseConfigResponse


class Armer:
    """
    The Manipulator Driver.

    :param robot: [description], defaults to None
    :type robot: rtb.robot.Robot, optional
    :param gripper: [description], defaults to None
    :type gripper: rtb.robot.Robot, optional
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

        if not self.robots:
            self.robots = [ROSRobot(self, rtb.models.URDF.UR5())]

        if not self.backend:
            self.backend = Swift()

        self.read_only_backends = []  # rtb.backends.Swift(realtime=False)]

        self.is_publishing_transforms = publish_transforms

        self.broadcaster: tf2_ros.TransformBroadcaster = None

        if self.is_publishing_transforms:
            self.broadcaster = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(500)

        # Load host specific arm configuration
        self.config_path: str = rospy.get_param('~config_path', os.path.join(
            os.getenv('HOME', '/root'),
            '.ros/configs/armer.yaml'
        ))
        self.custom_configs: List[str] = []

        self.__load_config()

        # Tooltip offsets
        if rospy.has_param('~tool_name'):
            self.tool_name = rospy.get_param('~tool_name')

        if rospy.has_param('~tool_offset'):
            self.tool_offset = rospy.get_param('~tool_offset')

        # Launch backend
        self.backend.launch()

        for robot in self.robots:
            self.backend.add(robot)

        for readonly in self.read_only_backends:
            readonly.launch()
            readonly.add(self.robot, readonly=True)

        rospy.Service('get_named_poses', GetNamedPoses,
                      self.get_named_poses_cb)

        rospy.Service('set_named_pose', AddNamedPose, self.add_named_pose_cb)
        rospy.Service('remove_named_pose', RemoveNamedPose,
                      self.remove_named_pose_cb)

        rospy.Service(
            'add_named_pose_config',
            AddNamedPoseConfig,
            self.add_named_pose_config_cb
        )
        rospy.Service(
            'remove_named_pose_config',
            RemoveNamedPoseConfig,
            self.remove_named_pose_config_cb
        )
        rospy.Service(
            'get_named_pose_configs',
            GetNamedPoseConfigs,
            self.get_named_pose_configs_cb
        )

    def close(self):
        """
        Close backend and stop action servers
        """
        self.backend.close()
        self.pose_server.need_to_terminate = True
        self.named_pose_server.need_to_terminate = True
        self.pose_servo_server.need_to_terminate = True

    def get_named_poses_cb(self, req: GetNamedPosesRequest) -> GetNamedPosesResponse:
        """
        ROS Service callback:
        Retrieves the list of named poses available to the arm

        :param req: An empty request
        :type req: GetNamesListRequest
        :return: The list of named poses available for the arm
        :rtype: GetNamesListResponse
        """
        return GetNamedPosesResponse(list(self.named_poses.keys()))

    def add_named_pose_cb(self, req: AddNamedPoseRequest) -> AddNamedPoseResponse:
        """
        ROS Service callback:
        Adds the current arm pose as a named pose and saves it to the host config

        :param req: The name of the pose as well as whether to overwrite if the pose already exists
        :type req: AddNamedPoseRequest
        :return: True if the named pose was written successfully otherwise false
        :rtype: AddNamedPoseResponse
        """
        if req.pose_name in self.named_poses and not req.overwrite:
            rospy.logerr('Named pose already exists.')
            return AddNamedPoseResponse(success=False)

        self.named_poses[req.pose_name] = self.robot.q.tolist()
        self.__write_config('named_poses', self.named_poses)

        return AddNamedPoseResponse(success=True)

    def remove_named_pose_cb(self, req: RemoveNamedPoseRequest) -> RemoveNamedPoseResponse:
        """
        ROS Service callback:
        Adds the current arm pose as a named pose and saves it to the host config

        :param req: The name of the pose as well as whether to overwrite if the pose already exists
        :type req: AddNamedPoseRequest
        :return: True if the named pose was written successfully otherwise false
        :rtype: AddNamedPoseResponse
        """
        if req.pose_name not in self.named_poses and not req.overwrite:
            rospy.logerr('Named pose does not exists.')
            return AddNamedPoseResponse(success=False)

        del self.named_poses[req.pose_name]
        self.__write_config('named_poses', self.named_poses)

        return AddNamedPoseResponse(success=True)

    def add_named_pose_config_cb(
            self,
            request: AddNamedPoseConfigRequest) -> AddNamedPoseConfigResponse:
        """[summary]

        :param request: [description]
        :type request: AddNamedPoseConfigRequest
        :return: [description]
        :rtype: AddNamedPoseConfigResponse
        """
        self.custom_configs.append(request.config_path)
        self.__load_config()
        return True

    def remove_named_pose_config_cb(
            self,
            request: RemoveNamedPoseConfigRequest) -> RemoveNamedPoseConfigResponse:
        """[summary]

        :param request: [description]
        :type request: AddNamedPoseRequest
        :return: [description]
        :rtype: [type]
        """
        if request.config_path in self.custom_configs:
            self.custom_configs.remove(request.config_path)
            self.__load_config()
        return True

    def get_named_pose_configs_cb(
            self,
            request: GetNamedPoseConfigsRequest) -> GetNamedPoseConfigsResponse:
        """[summary]

        :param request: [description]
        :type request: GetNamedPoseConfigsRequest
        :return: [description]
        :rtype: GetNamedPoseConfigsResponse
        """
        return self.custom_configs

    def __load_config(self):
        """[summary]
        """
        self.named_poses = {}
        for config_name in self.custom_configs:
            try:
                config = yaml.load(open(config_name))
                if config and 'named_poses' in config:
                    self.named_poses.update(config['named_poses'])
            except IOError:
                rospy.logwarn(
                    'Unable to locate configuration file: {}'.format(config_name))

        if os.path.exists(self.config_path):
            try:
                config = yaml.load(open(self.config_path),
                                   Loader=yaml.SafeLoader)
                if config and 'named_poses' in config:
                    self.named_poses.update(config['named_poses'])

            except IOError:
                pass

    def __write_config(self, key: str, value: Any):
        """[summary]

        :param key: [description]
        :type key: str
        :param value: [description]
        :type value: Any
        """
        if not os.path.exists(os.path.dirname(self.config_path)):
            os.makedirs(os.path.dirname(self.config_path))

        config = {}

        try:
            with open(self.config_path) as handle:
                current = yaml.load(handle.read())

                if current:
                    config = current

        except IOError:
            pass

        config.update({key: value})

        with open(self.config_path, 'w') as handle:
            handle.write(yaml.dump(config))

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
