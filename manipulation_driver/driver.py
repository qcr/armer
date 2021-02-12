#!/usr/bin/env python3
"""
Manipulation Driver Class

.. codeauthor:: Gavin Suddreys
"""
import os
import timeit

from threading import Lock, Event
from typing import List, Any

import rospy
import actionlib
import tf2_ros
import yaml

import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
from spatialmath.base.argcheck import getvector
import numpy as np

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist

from rv_msgs.msg import ManipulatorState
from rv_msgs.msg import JointVelocity
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal, MoveToPoseResult
from rv_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseGoal, MoveToNamedPoseResult
from rv_msgs.msg import ServoToPoseAction, ServoToPoseGoal, ServoToPoseResult
from rv_msgs.srv import GetNamesList, GetNamesListRequest, GetNamesListResponse
from rv_msgs.srv import SetNamedPose, SetNamedPoseRequest, SetNamedPoseResponse
from rv_msgs.srv import SetNamedPoseConfig, SetNamedPoseConfigRequest, SetNamedPoseConfigResponse
from rv_msgs.srv import GetNamedPoseConfigs, GetNamedPoseConfigsRequest, GetNamedPoseConfigsResponse

from .utils import populate_transform_stamped

class ManipulationDriver:
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
        robot: rtb.robot.Robot = None,
        gripper: rtb.robot.Robot = None,
        backend: rtb.backends.Connector=None,
        publish_transforms: bool=False) -> None:

        self.robot: rtb.robot.Robot = robot
        self.gripper: rtb.robot.Robot = gripper
        self.backend: rtb.backends.Connector = backend

        if not self.robot:
            self.robot = rtb.models.URDF.Panda()

        if not self.backend:
            self.backend = rtb.backends.Swift()

        self.is_publishing_transforms = publish_transforms

        self.broadcaster: tf2_ros.TransformBroadcaster = None

        if self.is_publishing_transforms:
            self.broadcaster = tf2_ros.TransformBroadcaster()

        # Guards used to prevent multiple motion requests conflicting
        self.moving: bool = False
        self.preempted: bool = False

        self.lock: Lock = Lock()
        self.event: Event = Event()

        # Load host specific arm configuration
        self.config_path: str = rospy.get_param('~config_path', os.path.join(
            os.getenv('HOME', '/root'),
            '.ros/configs/manipulation_driver.yaml'
        ))
        self.custom_configs: List[str] = []

        self.__load_config()

        # Arm state property
        self.state: ManipulatorState = ManipulatorState()

        self.e_v: np.array = np.zeros(shape=(6,)) # cartesian motion
        self.e_v_frame = None # The operating frame of the cartesian motion

        self.j_v: np.array = np.zeros(shape=(len(self.robot.q),)) # joint motion

        self.last_update: float = 0

        # Launch backend
        self.backend.launch()
        self.backend.add(self.robot)

        # Services
        rospy.Service('arm/home', Empty, self.home_cb)
        rospy.Service('arm/stop', Empty, self.preempt)

        rospy.Service('arm/get_named_poses', GetNamesList, self.get_named_poses_cb)
        rospy.Service('arm/set_named_pose', SetNamedPose, self.set_named_pose_cb)

        rospy.Service(
            'arm/add_named_pose_config',
            SetNamedPoseConfig,
            self.add_named_pose_config_cb
        )
        rospy.Service(
            'arm/remove_named_pose_config',
            SetNamedPoseConfig,
            self.remove_named_pose_config_cb
        )
        rospy.Service(
            'arm/get_named_pose_configs',
            GetNamedPoseConfigs,
            self.get_named_pose_configs_cb
        )

        # Publishers
        self.state_publisher: rospy.Publisher = rospy.Publisher(
            '/arm/state', ManipulatorState, queue_size=1
        )

        # Subscribers
        self.cartesian_velocity_subscriber: rospy.Subscriber = rospy.Subscriber(
            'arm/cartesian/velocity', TwistStamped, self.velocity_cb
        )
        self.joint_velocity_subscriber: rospy.Subscriber = rospy.Subscriber(
            'arm/joint/velocity', JointVelocity, self.joint_velocity_cb
        )

        # Action Servers
        self.pose_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            'arm/cartesian/pose',
            MoveToPoseAction,
            execute_cb=self.pose_cb,
            auto_start=False
        )
        self.pose_server.register_preempt_callback(self.preempt)
        self.pose_server.start()

        self.pose_servo_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            'arm/cartesian/servo_pose',
            ServoToPoseAction,
            execute_cb=self.servo_cb,
            auto_start=False
        )
        self.pose_servo_server.register_preempt_callback(self.preempt)
        self.pose_servo_server.start()

        self.named_pose_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            'arm/joint/named',
            MoveToNamedPoseAction,
            execute_cb=self.named_pose_cb,
            auto_start=False
        )
        self.named_pose_server.register_preempt_callback(self.preempt)
        self.named_pose_server.start()

    def close(self):
        """
        Close backend and stop action servers
        """
        self.backend.close()
        self.pose_server.need_to_terminate = True
        self.named_pose_server.need_to_terminate = True
        self.pose_servo_server.need_to_terminate = True

    def velocity_cb(self, msg: TwistStamped) -> None:
        """
        ROS Service callback:
        Moves the arm at the specified cartesian velocity
        w.r.t. a target frame

        :param msg: [description]
        :type msg: TwistStamped
        """
        if self.moving:
            self.preempt()

        with self.lock:
            target: Twist = msg.twist

            if msg.header.frame_id and msg.header.frame_id != self.robot.base_link.name:
                self.e_v_frame = msg.header.frame_id
            else:
                self.e_v_frame = None

            self.e_v = np.array([
                target.linear.x,
                target.linear.y,
                target.linear.z,
                target.angular.x,
                target.angular.y,
                target.angular.z
            ])
            self.last_update = timeit.default_timer()

    def joint_velocity_cb(self, msg: JointVelocity) -> None:
        """
        ROS Service callback:
        Moves the joints of the arm at the specified velocities

        :param msg: [description]
        :type msg: JointVelocity
        """
        if self.moving:
            self.preempt()

        with self.lock:
            self.j_v = np.array(msg.joints)
            self.last_update = timeit.default_timer()

    def pose_cb(self, goal: MoveToPoseGoal) -> None:
        """
        ROS Action Server callback:
        Moves the end-effector to the
        cartesian pose indicated by goal

        :param goal: [description]
        :type goal: MoveToPoseGoal
        """
        if self.moving:
            self.preempt()

        with self.lock:
            pose = goal.goal_pose.pose

            target = SE3(pose.position.x, pose.position.y, pose.position.z) * UnitQuaternion([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ]).SE3()

            q_pickup = self.robot.ikine_min(target, q0=self.robot.qr)
            traj = rtb.tools.trajectory.jtraj(self.robot.q, q_pickup.q, 50)

            if self.__traj_move(traj):
                self.pose_server.set_succeeded(MoveToPoseResult(result=0))
            else:
                self.pose_server.set_succeeded(MoveToPoseResult(result=1))

    def servo_cb(self, goal: ServoToPoseGoal) -> None:
        """
        ROS Action Server callback:
        Servos the end-effector to the cartesian pose indicated by goal

        :param goal: [description]
        :type goal: ServoToPoseGoal

        This callback makes use of the roboticstoolbox p_servo function
        to generate velocities at each timestep.
        """
        if self.moving:
            self.preempt()

        with self.lock:
            pose = goal.stamped_pose.pose

            target = SE3(pose.position.x, pose.position.y, pose.position.z) * UnitQuaternion([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ]).SE3()

            arrived = False

            self.moving = True

            while not arrived and not self.preempted:
                velocities, arrived = rtb.p_servo(self.robot.fkine(self.robot.q), target, 0.5)
                self.event.clear()
                self.robot.qd = np.linalg.pinv(self.robot.jacobe(self.robot.q)) @ velocities
                self.event.wait()

            self.moving = False
            result = not self.preempted
            self.preempted = False

            self.robot.qd *= 0

            self.pose_servo_server.set_succeeded(ServoToPoseResult(result=0 if result else 1))


    def named_pose_cb(self, goal: MoveToNamedPoseGoal) -> None:
        """
        ROS Action Server callback:
        Moves the arm the named pose indicated by goal

        :param goal: Goal message containing the name of
        the joint configuration to which the arm should move
        :type goal: MoveToNamedPoseGoal
        """
        if self.moving:
            self.preempt()

        with self.lock:
            if not goal.pose_name in self.named_poses:
                self.named_pose_server.set_aborted(
                    MoveToNamedPoseResult(result=1),
                    'Unknown named pose'
                )

            traj = rtb.tools.trajectory.jtraj(
                self.robot.q,
                np.array(self.named_poses[goal.pose_name]),
                50
            )

            self.__traj_move(traj)

            self.named_pose_server.set_succeeded(MoveToNamedPoseResult(result=0))

    def home_cb(self, req: EmptyRequest) -> EmptyResponse:
        """[summary]

        :param req: Empty request
        :type req: EmptyRequest
        :return: Empty response
        :rtype: EmptyResponse
        """
        if self.moving:
            self.preempt()

        with self.lock:
            traj = rtb.tools.trajectory.jtraj(self.robot.q, self.robot.qr, 50)
            self.__traj_move(traj)
            return EmptyResponse()

    def get_named_poses_cb(self, req: GetNamesListRequest) -> GetNamesListResponse:
        """
        ROS Service callback:
        Retrieves the list of named poses available to the arm

        :param req: An empty request
        :type req: GetNamesListRequest
        :return: The list of named poses available for the arm
        :rtype: GetNamesListResponse
        """
        return GetNamesListResponse(list(self.named_poses.keys()))

    def set_named_pose_cb(self, req: SetNamedPoseRequest) -> SetNamedPoseResponse:
        """
        ROS Service callback:
        Adds the current arm pose as a named pose and saves it to the host config

        :param req: The name of the pose as well as whether to overwrite if the pose already exists
        :type req: SetNamedPoseRequest
        :return: True if the named pose was written successfully otherwise false
        :rtype: SetNamedPoseResponse
        """
        if req.pose_name in self.named_poses and not req.overwrite:
            rospy.logerr('Named pose already exists.')
            return SetNamedPoseResponse(success=False)

        self.named_poses[req.pose_name] = self.robot.q.tolist()
        self.__write_config('named_poses', self.named_poses)

        return SetNamedPoseResponse(success=True)


    def add_named_pose_config_cb(
        self,
        request: SetNamedPoseConfigRequest) -> SetNamedPoseConfigResponse:
        """[summary]

        :param request: [description]
        :type request: SetNamedPoseConfigRequest
        :return: [description]
        :rtype: SetNamedPoseConfigResponse
        """
        self.custom_configs.append(request.config_path)
        self.__load_config()
        return True

    def remove_named_pose_config_cb(self, request: SetNamedPoseRequest):
        """[summary]

        :param request: [description]
        :type request: SetNamedPoseRequest
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


    def preempt(self, *args: list) -> None:
        """
        Stops any current motion
        """
        #pylint: disable=unused-argument
        self.preempted = True
        self.e_v *= 0
        self.j_v *= 0
        self.robot.qd *= 0

    def __traj_move(self, traj: np.array) -> bool:
        """[summary]

        :param traj: [description]
        :type traj: np.array
        :return: [description]
        :rtype: bool
        """
        self.moving = True
        for pose in traj.q:
            if self.preempted:
                break

            self.event.clear()
            self.robot.q = pose
            self.event.wait()

        self.moving = False
        result = not self.preempted
        self.preempted = False
        return result

    def publish_state(self) -> None:
        """[summary]
        """
        ee_pose = self.robot.fkine(self.robot.q)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'panda_link0'

        pose_stamped.pose.position.x = ee_pose.t[0]
        pose_stamped.pose.position.y = ee_pose.t[1]
        pose_stamped.pose.position.z = ee_pose.t[2]

        ee_rot = UnitQuaternion(ee_pose.R)

        pose_stamped.pose.orientation.w = ee_rot.A[0]
        pose_stamped.pose.orientation.x = ee_rot.A[1]
        pose_stamped.pose.orientation.y = ee_rot.A[2]
        pose_stamped.pose.orientation.z = ee_rot.A[3]

        state = ManipulatorState()
        state.ee_pose = pose_stamped
        state.joint_poses = list(self.robot.q)

        self.state_publisher.publish(state)


    def publish_transforms(self) -> None:
        """[summary]
        """
        if not self.is_publishing_transforms:
            return

        joint_positions = getvector(self.robot.q, self.robot.n)

        for link in self.robot.elinks:
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

        for gripper in self.robot.grippers:
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
                rospy.logwarn('Unable to locate configuration file: {}'.format(config_name))

        if os.path.exists(self.config_path):
            try:
                config = yaml.load(open(self.config_path), Loader=yaml.SafeLoader)
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

        config.update({ key: value })

        with open(self.config_path, 'w') as handle:
            handle.write(yaml.dump(config))

    def run(self) -> None:
        """
        Runs the driver. This is a blocking call.
        """
        while not rospy.is_shutdown():
            if any(self.e_v):
                if timeit.default_timer() - self.last_update > 1.0:
                    self.e_v = np.zeros(shape=self.e_v.shape)

                if self.e_v_frame:
                    self.robot.qd = np.linalg.pinv(
                        self.robot.jacobe(self.robot.q, end=self.e_v_frame)
                    ) @ self.e_v
                else:
                    self.robot.qd = np.linalg.pinv(self.robot.jacob0(self.robot.q)) @ self.e_v

            elif any(self.j_v):
                if timeit.default_timer() - self.last_update > 1.0:
                    self.j_v = np.zeros(shape=self.j_v.shape)
                self.robot.qd = self.j_v

            self.backend.step()
            self.event.set()


            self.publish_transforms()
            self.publish_state()

            rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('manipulator')
    manipulator = ManipulationDriver(publish_transforms=True)
    manipulator.run()
