"""
ROSRobot module defines the ROSRobot type

.. codeauthor:: Gavin Suddreys
"""
from typing import List
from threading import Lock, Event
import timeit

import rospy
import actionlib
import tf
import roboticstoolbox as rtb
import spatialmath as sp
from spatialmath import SE3, SO3, UnitQuaternion
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import TwistStamped, Twist
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from armer_msgs.msg import ManipulatorState, JointVelocity
from armer_msgs.msg import MoveToJointPoseAction, MoveToJointPoseGoal, MoveToJointPoseResult
from armer_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseGoal, MoveToNamedPoseResult
from armer_msgs.msg import MoveToPoseAction, MoveToPoseGoal, MoveToPoseResult
from armer_msgs.msg import ServoToPoseAction, ServoToPoseGoal, ServoToPoseResult

from armer_msgs.srv import SetCartesianImpedance, \
    SetCartesianImpedanceRequest, \
    SetCartesianImpedanceResponse

# pylint: disable=too-many-instance-attributes


class ROSRobot(rtb.ERobot):
    """
    The ROSRobot class wraps the rtb.ERobot implementing basic ROS functionality
    """

    def __init__(self,
                 driver,
                 robot: rtb.robot.Robot,
                 joint_state_topic: str = None,
                 joint_velocity_topic: str = None,
                 *args,
                 **kwargs):  # pylint: disable=unused-argument

        super().__init__(robot)
        self.__dict__.update(robot.__dict__)

        self.driver = driver

        self.joint_names = list(map(lambda link: link.name.replace(
            'link', 'joint'), filter(lambda link: link.isjoint, self.elinks)))
        self.joint_indexes = []

        self.q = self.qr

        self.tau = [0] * len(robot.q)

        # Guards used to prevent multiple motion requests conflicting
        self.moving: bool = False
        self.last_moving: bool = False

        self.preempted: bool = False

        self.lock: Lock = Lock()
        self.event: Event = Event()

        # Arm state property
        self.state: ManipulatorState = ManipulatorState()

        self.e_v_frame: str = None

        self.e_v: np.array = np.zeros(shape=(6,))  # cartesian motion
        self.j_v: np.array = np.zeros(
            shape=(len(self.q),)
        )  # joint motion

        self.e_p = self.fkine(self.q)

        self.last_update: float = 0
        self.last_tick: float = 0

        self.joint_subscriber = rospy.Subscriber(
            joint_state_topic if joint_state_topic else '/joint_states',
            JointState,
            self._state_cb
        )
        self.joint_publisher = rospy.Publisher(
            joint_velocity_topic
            if joint_velocity_topic
            else '/joint_velocity_node_controller/joint_velocity',
            JointVelocity,
            queue_size=1
        )

        # Create Transform Listener
        self.tf_listener = tf.TransformListener()

        # Services
        rospy.Service('{}/home'.format(self.name.lower()), Empty, self.home_cb)
        rospy.Service('{}/recover'.format(self.name.lower()),
                      Empty, self.recover_cb)
        rospy.Service('{}/stop'.format(self.name.lower()), Empty, self.preempt)

        # Publishers
        self.state_publisher: rospy.Publisher = rospy.Publisher(
            '{}/state'.format(self.name.lower()), ManipulatorState, queue_size=1
        )

        # Subscribers
        self.cartesian_velocity_subscriber: rospy.Subscriber = rospy.Subscriber(
            '{}/cartesian/velocity'.format(self.name.lower()
                                           ), TwistStamped, self.velocity_cb
        )
        self.joint_velocity_subscriber: rospy.Subscriber = rospy.Subscriber(
            '{}/joint/velocity'.format(self.name.lower()
                                       ), JointVelocity, self.joint_velocity_cb
        )

        # Action Servers
        self.pose_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            '{}/cartesian/pose'.format(self.name.lower()),
            MoveToPoseAction,
            execute_cb=self.pose_cb,
            auto_start=False
        )
        self.pose_server.register_preempt_callback(self.preempt)
        self.pose_server.start()

        self.pose_servo_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            '{}/cartesian/servo_pose'.format(self.name.lower()),
            ServoToPoseAction,
            execute_cb=self.servo_cb,
            auto_start=False
        )
        self.pose_servo_server.register_preempt_callback(self.preempt)
        self.pose_servo_server.start()

        self.joint_pose_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            '{}/joint/pose'.format(self.name.lower()),
            MoveToJointPoseAction,
            execute_cb=self.joint_pose_cb,
            auto_start=False
        )
        self.joint_pose_server.register_preempt_callback(self.preempt)
        self.joint_pose_server.start()

        self.named_pose_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            '{}/joint/named'.format(self.name.lower()),
            MoveToNamedPoseAction,
            execute_cb=self.named_pose_cb,
            auto_start=False
        )
        self.named_pose_server.register_preempt_callback(self.preempt)
        self.named_pose_server.start()

        rospy.Service(
            '{}/set_cartesian_impedance'.format(self.name.lower()),
            SetCartesianImpedance,
            self.set_cartesian_impedance_cb
        )

    def _state_cb(self, msg):
        if not self.joint_indexes:
            self.joint_indexes = [idx for idx, joint_name in enumerate(
                msg.name) if joint_name in self.joint_names]

        self.tau = np.array(msg.effort)[self.joint_indexes]
        self.q = np.array(msg.position)[self.joint_indexes]

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

            if msg.header.frame_id and msg.header.frame_id != self.base_link.name:
                self.e_v_frame = msg.header.frame_id
            else:
                self.e_v_frame = None

            e_v = np.array([
                target.linear.x,
                target.linear.y,
                target.linear.z,
                target.angular.x,
                target.angular.y,
                target.angular.z
            ])

            if np.any(e_v - self.e_v):
                self.e_p = self.fkine(self.q, fast=True)

            self.e_v = e_v

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
            goal_pose = goal.pose_stamped

            if goal_pose.header.frame_id == '':
                goal_pose.header.frame_id = self.base_link.name

            goal_pose = self.tf_listener.transformPose(
                self.base_link.name,
                goal_pose
            )
            pose = goal_pose.pose

            target = SE3(pose.position.x, pose.position.y, pose.position.z) * UnitQuaternion([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ]).SE3()

            dq = self.ikine_LMS(target, q0=self.q)
            traj = rtb.tools.trajectory.jtraj(self.q, dq.q, 100)

            if self.__traj_move(traj, goal.speed if goal.speed else 0.4):
                self.pose_server.set_succeeded(MoveToPoseResult(success=True))
            else:
                self.pose_server.set_aborted(MoveToPoseResult(success=False))

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
            goal_pose = goal.pose_stamped

            if goal_pose.header.frame_id == '':
                goal_pose.header.frame_id = self.base_link.name

            goal_pose = self.tf_listener.transformPose(
                self.base_link.name,
                goal_pose
            )
            pose = goal_pose.pose

            target = SE3(pose.position.x, pose.position.y, pose.position.z) * UnitQuaternion([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ]).SE3()

            arrived = False

            self.moving = True

            while not arrived and not self.preempted:
                velocities, arrived = rtb.p_servo(
                    self.fkine(self.q),
                    target,
                    min(3, goal.gain) if goal.gain else 2,
                    threshold=goal.threshold if goal.threshold else 0.005
                )
                self.event.clear()
                self.j_v = np.linalg.pinv(
                    self.jacobe(self.q)) @ velocities
                self.last_update = timeit.default_timer()
                self.event.wait()

            self.moving = False
            result = not self.preempted
            self.preempted = False

            # self.qd *= 0
            if result:
                self.pose_servo_server.set_succeeded(
                    ServoToPoseResult(success=True)
                )
            else:
                self.pose_servo_server.set_aborted(
                    ServoToPoseResult(success=False)
                )

    def joint_pose_cb(self, goal: MoveToJointPoseGoal) -> None:
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
            traj = rtb.tools.trajectory.jtraj(
                self.q,
                np.array(goal.joints),
                100,
            )

            if self.__traj_move(traj, goal.speed if goal.speed else 0.4):
                self.named_pose_server.set_succeeded(
                    MoveToJointPoseResult(success=True)
                )
            else:
                self.joint_pose_server.set_aborted(
                    MoveToJointPoseResult(success=False)
                )

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
            if not goal.pose_name in self.driver.named_poses:
                self.named_pose_server.set_aborted(
                    MoveToNamedPoseResult(success=False),
                    'Unknown named pose'
                )

            traj = rtb.tools.trajectory.jtraj(
                self.q,
                np.array(self.driver.named_poses[goal.pose_name]),
                100
            )

            if self.__traj_move(traj, goal.speed if goal.speed else 0.4):
                self.named_pose_server.set_succeeded(
                    MoveToNamedPoseResult(success=True)
                )
            else:
                self.named_pose_server.set_aborted(
                    MoveToNamedPoseResult(success=False)
                )

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
            traj = rtb.tools.trajectory.jtraj(self.q, self.qr, 200)
            self.__traj_move(traj)
            return EmptyResponse()

    def recover_cb(self, req: EmptyRequest) -> EmptyResponse:
        """[summary]
        ROS Service callback:
        Invoke any available error recovery functions on the robot when an error occurs

        :param req: an empty request
        :type req: EmptyRequest
        :return: an empty response
        :rtype: EmptyResponse
        """
        rospy.logwarn('Recovery not implemented for this arm')
        return EmptyResponse()

    def set_cartesian_impedance_cb(
            self,
            request: SetCartesianImpedanceRequest) -> SetCartesianImpedanceResponse:
        """
        ROS Service Callback
        Set the 6-DOF impedance of the end-effector. Higher values should increase the stiffness
        of the robot while lower values should increase compliance

        :param request: The numeric values representing the EE impedance (6-DOF) that
        should be set on the arm
        :type request: GetNamedPoseConfigsRequest
        :return: True if the impedence values were updated successfully
        :rtype: GetNamedPoseConfigsResponse
        """
        rospy.logwarn(
            'Setting cartesian impedance not implemented for this arm')
        return SetCartesianImpedanceResponse(True)

    def preempt(self, *args: list) -> None:
        """
        Stops any current motion
        """
        # pylint: disable=unused-argument
        self.preempted = True
        self.e_v *= 0
        self.j_v *= 0
        self.qd *= 0

    def __traj_move(self, traj: np.array, max_speed=0.4) -> bool:
        """[summary]

        :param traj: [description]
        :type traj: np.array
        :return: [description]
        :rtype: bool
        """
        Kp = 15
        self.moving = True

        for dq in traj.q[1:]:
            if self.preempted:
                break

            error = [-1] * self.n
            while np.max(np.fabs(error)) > 0.05 and not self.preempted:
                error = dq - self.q

                jV = Kp * error

                jacob0 = self.jacob0(self.q, fast=True)

                T = jacob0 @ jV
                V = np.linalg.norm(T[:3])

                if V > max_speed:
                    T = T / V * max_speed
                    jV = (np.linalg.pinv(jacob0) @ T)

                self.event.clear()
                self.j_v = jV
                self.last_update = timeit.default_timer()
                self.event.wait()

        self.moving = False
        result = not self.preempted
        self.preempted = False
        return result

    def publish_state(self) -> None:
        """[summary]
        """
        """
        Generates a ManipulatorState message for the robot

        :return: ManipulatorState message describing the current state of the robot
        :rtype: ManipulatorState
        """
        ee_pose = self.fkine(self.q)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_link.name

        pose_stamped.pose.position.x = ee_pose.t[0]
        pose_stamped.pose.position.y = ee_pose.t[1]
        pose_stamped.pose.position.z = ee_pose.t[2]

        ee_rot = sp.UnitQuaternion(ee_pose.R)

        pose_stamped.pose.orientation.w = ee_rot.A[0]
        pose_stamped.pose.orientation.x = ee_rot.A[1]
        pose_stamped.pose.orientation.y = ee_rot.A[2]
        pose_stamped.pose.orientation.z = ee_rot.A[3]

        state = ManipulatorState()
        state.ee_pose = pose_stamped
        state.joint_poses = list(self.q)
        state.joint_torques = list(self.tau)

        self.state_publisher.publish(state)

    def step(self, dt: float = 0.01) -> None:  # pylint: disable=unused-argument
        """
        Updates the robot joints (robot.q) used in computing kinematics
        :param dt: the delta time since the last update, defaults to 0.01
        :type dt: float, optional
        """
        current_time = timeit.default_timer()

        Kp = 1

        self.publish_state()

        # calculate joint velocities from desired cartesian velocity
        if any(self.e_v):
            if current_time - self.last_update > 0.1:
                self.e_v *= 0.9 if np.sum(np.absolute(self.e_v)
                                          ) >= 0.0001 else 0

            wTe = self.fkine(self.q, fast=True)
            error = self.e_p @ np.linalg.inv(wTe)
            # print(e)
            trans = self.e_p[:3, 3]  # .astype('float64')
            trans += self.e_v[:3] * dt
            rotation = SO3(self.e_p[:3, :3])

            # Rdelta = SO3.EulerVec(self.e_v[3:])

            # R = Rdelta * R
            # R = R.norm()
            # # print(self.e_p)
            self.e_p = SE3.Rt(rotation, t=trans).A

            v_t = self.e_v[:3] + Kp * error[:3, 3]
            v_r = self.e_v[3:]  # + (e[.rpy(]) * 0.5)

            e_v = np.concatenate([v_t, v_r])

            self.j_v = np.linalg.pinv(
                self.jacob0(self.q, fast=True)) @ e_v

        # apply desired joint velocity to robot
        if any(self.j_v):
            if current_time - self.last_update > 0.1:
                self.j_v *= 0.9 if np.sum(np.absolute(self.j_v)
                                          ) >= 0.0001 else 0

            self.qd = self.j_v

        self.joint_publisher.publish(JointVelocity(joints=self.qd))
        self.last_tick = current_time

        self.event.set()
