#!/usr/bin/env python3
"""ROSRobot module defines the ROSRobot type

Defines the callbacks and underlying function to control a roboticstoolbox robot
"""

from __future__ import annotations

__author__ = ['Gavin Suddrey', 'Dasun Gunasinghe']
__version__ = "0.1.0"

import os
import rclpy
import tf2_ros
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import yaml
import time
# Required for NEO
import qpsolvers as qp
import spatialgeometry as sg

from typing import List, Any
from threading import Lock, Event
from armer.models.URDFRobot import URDFRobot
from armer.utils import mjtg, ikine
from armer.errors import ArmerError
from armer.trajectory import TrajectoryExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# pylint: disable=too-many-instance-attributes

# ROS Message Imports
from armer_msgs.msg import ManipulatorState, JointVelocity, ServoStamped, Guards
from armer_msgs.action import GuardedVelocity, Home, MoveToJointPose, MoveToNamedPose, MoveToPose
from armer_msgs.srv import GetNamedPoses, AddNamedPose 
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped
from std_msgs.msg import Header, Float64MultiArray, Bool
from sensor_msgs.msg import JointState

class ControlMode:
   JOINTS=1
   CARTESIAN=2

class ROSRobot(URDFRobot):
    """The ROSRobot class wraps the URDFRobot implementing basic ROS functionality
    """
    def __init__(self,
                 nh=None,
                 name: str = None,
                 joint_state_topic: str = None,
                 joint_velocity_topic: str = None,
                 origin=None,
                 config_path=None,
                 readonly=False,
                 frequency=None,
                 modified_qr=None,
                 singularity_thresh=0.02,
                 action_cb_group=None,
                 * args,
                 **kwargs):  # pylint: disable=unused-argument
        
        super().__init__(nh, *args, **kwargs)
        
        # Setup the name of the robot
        self.name = name if name else self.name
        # Configure action servers with their required callback group
        self.action_cb_group = action_cb_group if action_cb_group else ReentrantCallbackGroup()
        # TESTING for NEO
        self.collision_obj_list: List[sg.Shape] = list()
        
        # Handle the setup of joint stat and velocity topics for a ROS backend
        # NOTE: this should match the ros_control real robot state and control (group velocity) interface topics
        #       the defaults are generally typical, but could be different
        self.joint_state_topic = joint_state_topic \
                if joint_state_topic \
                else '/joint_states'

        self.joint_velocity_topic = joint_velocity_topic \
                if joint_velocity_topic \
                else '/joint_group_velocity_controller/command'

        # Specify the path to named pose configuration
        # NOTE: this is a yaml to load/save named poses
        #       the default is in the root path (for shared usage)
        #       can be configured for project specific loading (see service)
        self.config_path = config_path if config_path else os.path.join(
            os.getenv('HOME', '/root'),
            '.ros/configs/armer.yaml'
        )

        # Configure the gripper name
        if not hasattr(self, 'gripper'):
            self.gripper = self.grippers[0].name
          
        # Sort links by parents starting from gripper
        sorted_links=[]
        link=self.link_dict[self.gripper]   
        while link is not None:
            sorted_links.append(link)
            link=link.parent
        sorted_links.reverse()

        self.joint_indexes = []
        self.joint_names = list(map(lambda link: link._joint_name, filter(lambda link: link.isjoint, sorted_links)))
        
        # Configure a new origin if specified
        if origin:
            self.base = sm.SE3(origin[:3]) @ sm.SE3.RPY(origin[3:])

        # Configure the frequency of operation (step method) based on input
        # NOTE: defaulting to 500Hz
        # TODO: currently getting from a defined param - is this correct?
        self.frequency = frequency if frequency else self.get_parameter(
          f'{self.joint_state_topic}/frequency', 500
        )
        self.logger(f"Configured frequency (step): {self.frequency}")
        
        # Setup the robot's default ready state (or modified if specified)
        self.q = self.qr if hasattr(self, 'qr') else self.q # pylint: disable=no-member
        if modified_qr:
            self.qr = modified_qr
            self.q = modified_qr

        # Joint state message
        self.joint_states = None 

        # Guards used to prevent multiple motion requests conflicting
        self._controller_mode = ControlMode.JOINTS

        # Flag initialisation
        self.moving: bool = False
        self.last_moving: bool = False
        self.preempted: bool = False

        # Thread variable setup
        self.lock: Lock = Lock()
        self.event: Event = Event()

        # Arm state property setup
        self.state: ManipulatorState = ManipulatorState()
        # Expected cartesian velocity, frame and joint velocity initialisation
        self.e_v_frame: str = None
        self.e_v: np.array = np.zeros(shape=(6,)) 
        self.j_v: np.array = np.zeros(
            shape=(len(self.q),)
        )

        # Expected position of end-effector initialisation
        self.e_p = self.fkine(self.q, start=self.base_link, end=self.gripper)

        # Timer variable setup
        self.last_update: float = 0
        self.last_tick: float = 0

        # Trajectory executor and generator initialisation
        # NOTE: default is a minimum jerk trajectory (see armer.utils)
        #       this can be replaced by any generator as needed
        # TODO: introduce mechanism to update generator and solver as needed
        self.executor = None        
        self.traj_generator = mjtg
        self.ik_solver = ikine

        # Configure ROS transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        if hasattr(self.nh, 'get_clock'):
          self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self.nh)
        else:
          self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer)

        self.readonly = readonly

        self.custom_configs: List[str] = []

        # Singularity index threshold (0 is a sigularity)
        # NOTE: this is a tested value and may require configuration (i.e., speed of robot)
        self.logger(f"Singularity Scalar Threshold set to: {singularity_thresh}")
        self.singularity_thresh = singularity_thresh 
        self.manip_scalar = None
        self.singularity_approached = False

        # Loads a configured named pose config 
        self.__load_named_pose_config()

        #### --- ROS SETUP --- ###
        self.nh.create_subscription(
          JointState,
          self.joint_state_topic,
          self.robot_state_cb,
          1
        )
        
        # Setup interfaces for non-readonly
        if not self.readonly:
            # -- Publishers -- #
            # NOTE: the joint publisher topic should attach to the ros_control input for real robot
            self.joint_publisher = self.nh.create_publisher(
                Float64MultiArray,
                self.joint_velocity_topic,
                1
            )
            self.state_publisher = self.nh.create_publisher(
                ManipulatorState, 
                '{}/state'.format(self.name.lower()), 
                1
            )
            self.cartesian_servo_publisher = self.nh.create_publisher(
                Bool, 
                '{}/cartesian/servo/arrived'.format(self.name.lower()), 
                1
            )
                    
            # -- Subscribers -- #
            self.cartesian_velocity_subscriber = self.nh.create_subscription(
                TwistStamped,
                '{}/cartesian/velocity'.format(self.name.lower()), 
                self.cartesian_velocity_cb,
                10
            )
            self.joint_velocity_subscriber = self.nh.create_subscription(
                JointVelocity,
                '{}/joint/velocity'.format(self.name.lower()), 
                self.joint_velocity_cb,
                10
            )
            self.cartesian_servo_subscriber = self.nh.create_subscription(
                ServoStamped,
                '{}/cartesian/servo'.format(self.name.lower()), 
                self.servo_cb,
                10
            )

            # -- Action Servers -- #
            self.move_pose_action = rclpy.action.ActionServer(
                node=self.nh,
                action_type=MoveToPose,
                action_name='{}/cartesian/pose'.format(self.name.lower()),
                execute_callback=self.cartesian_pose_cb,
                callback_group=self.action_cb_group
            )
            self.home_action = rclpy.action.ActionServer(
                node=self.nh,
                action_type=Home,
                action_name='{}/home'.format(self.name.lower()),
                execute_callback=self.home_cb,
                callback_group=self.action_cb_group
            )
            self.guarded_velocity_action = rclpy.action.ActionServer(
                node=self.nh,
                action_type=GuardedVelocity,
                action_name='{}/cartesian/guarded_velocity'.format(self.name.lower()),
                execute_callback=self.guarded_velocity_cb,
                callback_group=self.action_cb_group
            )
            self.move_joint_action = rclpy.action.ActionServer(
                node=self.nh,
                action_type=MoveToJointPose,
                action_name='{}/joint/pose'.format(self.name.lower()),
                execute_callback=self.joint_pose_cb,
                callback_group=self.action_cb_group
            )
            self.move_named_pose_action = rclpy.action.ActionServer(
                node=self.nh,
                action_type=MoveToNamedPose,
                action_name='{}/joint/named'.format(self.name.lower()),
                execute_callback=self.named_pose_cb,
                callback_group=self.action_cb_group
            )

            # -- Services -- #
            self.get_named_pose_srv = self.nh.create_service(
                GetNamedPoses, 
                '{}/get_named_poses'.format(self.name.lower()), 
                self.get_named_poses_cb
            )
            self.set_named_pose_srv = self.nh.create_service(
                AddNamedPose, 
                '{}/set_named_pose'.format(self.name.lower()), 
                self.add_named_pose_cb
            )

        #     rospy.Service('{}/recover'.format(self.name.lower()),
        #                   Empty, self.recover_cb)
        #     rospy.Service('{}/stop'.format(self.name.lower()),
        #                   Empty, self.preempt)

        #     rospy.Service(
        #         '{}/set_cartesian_impedance'.format(self.name.lower()),
        #         SetCartesianImpedance,
        #         self.set_cartesian_impedance_cb
        #     )

        #     rospy.Service(
        #         '{}/get_ee_link_name'.format(self.name.lower()),
        #         GetLinkName,
        #         lambda req: GetLinkNameResponse(name=self.gripper)
        #     )
            
        #     rospy.Service(
        #         '{}/get_base_link_name'.format(self.name.lower()),
        #         GetLinkName,
        #         lambda req: GetLinkNameResponse(name=self.base_link.name)
        #     )

        #     rospy.Service('{}/set_named_pose'.format(self.name.lower()), AddNamedPose,
        #                   self.add_named_pose_cb)
        #     rospy.Service('{}/remove_named_pose'.format(self.name.lower()), RemoveNamedPose,
        #                   self.remove_named_pose_cb)

        #     rospy.Service(
        #         '{}/add_named_pose_config'.format(self.name.lower()),
        #         AddNamedPoseConfig,
        #         self.add_named_pose_config_cb
        #     )
        #     rospy.Service(
        #         '{}/remove_named_pose_config'.format(self.name.lower()),
        #         RemoveNamedPoseConfig,
        #         self.remove_named_pose_config_cb
        #     )
        #     rospy.Service(
        #         '{}/get_named_pose_configs'.format(self.name.lower()),
        #         GetNamedPoseConfigs,
        #         self.get_named_pose_configs_cb
        #     )

    # --------------------------------------------------------------------- #
    # --------- ROS Topic Callback Methods -------------------------------- #
    # --------------------------------------------------------------------- #
    def robot_state_cb(self, msg):
        """ROS Robot State Callback
        
        - when configured to a real robot gets real state info 
        """
        if not self.joint_indexes:
            for joint_name in self.joint_names:
                self.joint_indexes.append(msg.name.index(joint_name))
        
        self.q = np.array(msg.position)[self.joint_indexes] if len(msg.position) == self.n else np.zeros(self.n)
        self.joint_states = msg
        
    def cartesian_velocity_cb(self, msg: TwistStamped) -> None:
        """ROS Cartesian Velocity Subscriber Callback:
        
        Moves the arm at the specified cartesian velocity
        w.r.t. a target frame

        :param msg: [description]
        :type msg: TwistStamped
        """
        if self.moving:
            self.preempt()

        with self.lock:
            self.preempted = False
            self.__vel_move(msg)

    def joint_velocity_cb(self, msg: JointVelocity) -> None:
        """ROS Joint Velocity Subscriber Callback:
        
        Moves the joints of the arm at the specified joint velocities

        :param msg: [description]
        :type msg: JointVelocity
        """
        if self.moving:
            self.preempt()

        with self.lock:
            self.j_v = np.array(msg.joints)
            self.last_update = self.get_time()

    def servo_cb(self, msg: ServoStamped) -> None:
        """ROS Cartesian Servo Subscriber Callback:
        
        Servos the end-effector to the cartesian pose given by msg
        
        :param msg: [description]
        :type msg: PoseStamped

        This callback makes use of the roboticstoolbox p_servo function
        to generate velocities at each timestep.
        """
        # Safely stop any current motion of the arm
        if self.moving:
            self.preempt()
        
        with self.lock:
            # Setup servo parameters
            goal_pose = msg.pose
            goal_gain = msg.gain if msg.gain else 0.5
            goal_thresh = msg.threshold if msg.threshold else 0.005
            # Configure servo flags
            arrived = False
            self.moving = True
            self.preempted = False

            # Current end-effector pose
            Te = self.ets(start=self.base_link, end=self.gripper).eval(self.q)

            # Default frame to base_link if not specified
            if msg.header.frame_id == '':
                msg.header.frame_id = self.base_link.name
            
            # Construct a target pose (SE3) for servo
            goal_pose_stamped = self.transform(
                PoseStamped(header=msg.header, pose=goal_pose),
                self.base_link.name
            )
            pose = goal_pose_stamped.pose
            U = sm.UnitQuaternion([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ]).SE3()
            target = sm.SE3(pose.position.x, pose.position.y, pose.position.z) * U

            # Calculate joint velocities using servo feature
            # NOTE: maximum gain currently capped at 5 
            velocities, arrived = rtb.p_servo(
                wTe=Te,
                wTep=target,
                gain=min(5, goal_gain),
                threshold=goal_thresh
            )

            ##### TESTING NEO IMPLEMENTATION #####
            # neo_jv = self.neo(Tep=target, velocities=velocities)
            neo_jv = None

            if np.any(neo_jv):
                self.j_v = neo_jv[:len(self.q)]
            else:
                self.j_v = np.linalg.pinv(self.jacobe(self.q)) @ velocities

            self.last_update = self.get_time()

        # Configure arrived (bool) to std_msgs Bool type for publish
        arrived_out = Bool()
        arrived_out.data = arrived
        self.cartesian_servo_publisher.publish(arrived_out)

    # --------------------------------------------------------------------- #
    # --------- ROS Action Callback Methods ------------------------------- #
    # --------------------------------------------------------------------- #
    def guarded_velocity_cb(self, goal_handle) -> None:
        """ROS Action Server callback:
        Moves the end-effector to the
        cartesian pose indicated by goal (defined by guards in twist or wrench)

        :param goal: [description]
        :type goal: GuardedVelocity
        """
        request = goal_handle._goal_request

        # Check for movement and preempt
        if self.moving:
            self.preempt()
        
        with self.lock:
            self.preempted = False
            
            start_time = self.get_time()
            triggered = 0
            
            while not self.preempted:
                triggered = self.test_guards(request.guards, start_time=start_time)

                if triggered != 0:
                    break

                self.__vel_move(request.twist_stamped)
                time.sleep(0.01)

            # Set action server outputs
            goal_handle.succeed()
            self.moving = False

        # Finalise result
        action_result = GuardedVelocity.Result()
        return action_result
    
    def cartesian_pose_cb(self, goal_handle):
        """ROS Action Server callback:
        Moves the end-effector to the
        cartesian pose indicated by goal

        :param goal: [description]
        :type goal: MoveToPose
        """
        request = goal_handle._goal_request

        # Check for movement and preempt
        if self.moving:
            self.preempt()

        with self.lock:
            # Take requested pose goal and resolve for execution
            goal_pose = request.pose_stamped
            if goal_pose.header.frame_id == '':
                goal_pose.header.frame_id = self.base_link.name
            goal_pose = self.transform(goal_pose, self.base_link.name)
            pose = goal_pose.pose
            
            # Calculate solution to pose and execute configured trajectory type
            solution = ikine(self, pose, q0=self.q, end=self.gripper)
            self.executor = TrajectoryExecutor(
              self,
              self.traj_generator(self, solution.q, request.speed if request.speed else 0.2)
            )

            # Wait for finish (NOTE: this is a Reentrant Callback, so the step method updates)
            while not self.executor.is_finished():
              time.sleep(0.01)

            # Set action server outputs
            result = self.executor.is_succeeded()
            if result:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            self.executor = None
            self.moving = False

        # Finalise result
        action_result = MoveToPose.Result()
        action_result.success = result

        # DEBUGGING
        # self.logger(f"action result: {action_result.success} | result: {result}")

        return action_result

    def joint_pose_cb(self, goal_handle):
        """ROS Action Server callback:
        
        Moves the arm the named pose indicated by goal

        :param goal: Goal message containing the name of
        the joint configuration to which the arm should move
        :type goal: MoveToJointPose
        """
        request = goal_handle._goal_request

        if self.moving:
            self.preempt()

        with self.lock:     

            # Handle user request joint array miss-match
            if len(self.q) != len(request.joints):
                self.logger(f"Requested joints number miss-match: {len(request.joints)} | required: {len(self.q)}", 'error')
                goal_handle.abort()
                result = False
            else:
                # Prepare executor with request joints as goal (at requested speed)
                self.executor = TrajectoryExecutor(
                    self,
                    self.traj_generator(self, np.array(request.joints), request.speed if request.speed else 0.2)
                )

                while not self.executor.is_finished():
                    time.sleep(0.01)

                # Set action server outputs
                result = self.executor.is_succeeded()
                if result:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()

                self.executor = None
                self.moving = False

        # Finalise result
        action_result = MoveToJointPose.Result()
        action_result.success = result

        # DEBUGGING
        # self.logger(f"action result: {action_result.success} | result: {result}")

        return action_result

    def named_pose_cb(self, goal_handle):
        """
        ROS Action Server callback:
        Moves the arm the named pose indicated by goal

        :param goal: Goal message containing the name of
        the joint configuration to which the arm should move
        :type goal: MoveToNamedPose
        """
        request = goal_handle._goal_request

        if self.moving:
            self.preempt()

        with self.lock:
            if not request.pose_name in self.named_poses:
              raise ArmerError('Unknown named pose')

            qd = np.array(self.named_poses[request.pose_name])

            self.executor = TrajectoryExecutor(
                self,
                self.traj_generator(self, qd, request.speed if request.speed else 0.2)
            )

            while not self.executor.is_finished():
                time.sleep(0.01)

            # Set action server outputs
            result = self.executor.is_succeeded()
            if result:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            self.executor = None
            self.moving = False

        # Finalise result
        action_result = MoveToNamedPose.Result()
        action_result.success = result

        # DEBUGGING
        # self.logger(f"action result: {action_result.success} | result: {result}")

        return action_result

    def home_cb(self, goal_handle) -> None:
        """ROS Action to Send Robot Home

        :param req: Empty request
        :type req: EmptyRequest
        :return: Empty response
        :rtype: EmptyResponse
        """
        request = goal_handle._goal_request

        # Check for movement and preempt
        if self.moving:
            self.preempt()
            
        with self.lock:
            # Get the configured ready state (joints) of the arm and execute configured trajectory type
            qd = np.array(self.qr) if hasattr(self, 'qr') else self.q
            self.executor = TrajectoryExecutor(
                self,
                self.traj_generator(self, qd, request.speed if request.speed else 0.2)
            )

            # Wait for finish (NOTE: this is a Reentrant Callback, so the step method updates)
            while not self.executor.is_finished():
                time.sleep(0.01)

            # Set action server outputs
            result = self.executor.is_succeeded()
            if result:
                goal_handle.succeed()
            else:
                goal_handle.abort()
                
            self.executor = None
            self.moving = False

        # Finalise result
        action_result = Home.Result()
        action_result.success = result

        # DEBUGGING
        # self.logger(f"action result: {action_result.success} | result: {result}")

        return action_result

    # --------------------------------------------------------------------- #
    # --------- ROS Service Callback Methods ------------------------------ #
    # --------------------------------------------------------------------- #
    def recover_cb(self, req): # pylint: disable=no-self-use
        """[summary]
        ROS Service callback:
        Invoke any available error recovery functions on the robot when an error occurs

        :param req: an empty request
        :type req: EmptyRequest
        :return: an empty response
        :rtype: EmptyResponse
        """
        return NotImplementedError()

    def set_cartesian_impedance_cb(self, request): # pylint: disable=no-self-use
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
        return NotImplementedError()
    
    def get_named_poses_cb(self, request, response):
        """
        ROS Service callback:
        Retrieves the list of named poses available to the arm

        :param request: An empty request
        :type request: GetNamesListRequest
        :return: The list of named poses available for the arm
        :rtype: GetNamesListResponse
        """
        # print(f"response attr: {response.__dir__()}")
        response.names_list = list(self.named_poses.keys())
        return response
        
    def add_named_pose_cb(self, request, response):
        """
        ROS Service callback:
        Adds the current arm pose as a named pose and saves it to the host config

        :param request: The name of the pose as well as whether to overwrite if the pose already exists
        :type request: AddNamedPoseRequest
        :return: True if the named pose was written successfully otherwise false
        :rtype: AddNamedPoseResponse
        """
        if request.pose_name in self.named_poses and not request.overwrite:
            self.logger('Named pose already exists.', 'warn')
            response.success=False
        else:
            self.named_poses[request.pose_name] = self.q.tolist()
            self.__write_config('named_poses', self.named_poses)
            response.success=True

        return response

    def remove_named_pose_cb(self, req):
        """
        ROS Service callback:
        Adds the current arm pose as a named pose and saves it to the host config

        :param req: The name of the pose as well as whether to overwrite if the pose already exists
        :type req: AddNamedPoseRequest
        :return: True if the named pose was written successfully otherwise false
        :rtype: AddNamedPoseResponse
        """
        raise NotImplementedError()

    def add_named_pose_config_cb(self, request):
        """[summary]

        :param request: [description]
        :type request: AddNamedPoseConfigRequest
        :return: [description]
        :rtype: AddNamedPoseConfigResponse
        """
        raise NotImplementedError()

    def remove_named_pose_config_cb(self, request):
        """[summary]

        :param request: [description]
        :type request: AddNamedPoseRequest
        :return: [description]
        :rtype: [type]
        """
        raise NotImplementedError()

    def get_named_pose_configs_cb(self, request):
        """[summary]

        :param request: [description]
        :type request: GetNamedPoseConfigsRequest
        :return: [description]
        :rtype: GetNamedPoseConfigsResponse
        """
        return self.custom_configs
    
    # --------------------------------------------------------------------- #
    # --------- Standard Methods ------------------------------------------ #
    # --------------------------------------------------------------------- #
    def close(self):
        """
        Closes any resources associated with this robot
        """
        self.pose_server.need_to_terminate = True
        self.joint_pose_server.need_to_terminate = True
        self.named_pose_server.need_to_terminate = True

    def add_collision_obj(self, obj: sg.Shape):
        self.collision_obj_list.append(obj)

    def neo(self, Tep, velocities):
        """
        Runs a version of Jesse H.'s NEO controller
        <IN DEVELOPMENT
        """
        ##### Determine Slack #####
        # Transform from the end-effector to desired pose
        Te = self.fkine(self.q)
        eTep = Te.inv() * Tep
        # Spatial error
        e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))

        # Gain term (lambda) for control minimisation
        Y = 0.01

        # Quadratic component of objective function
        Q = np.eye(len(self.q) + 6)

        # Joint velocity component of Q
        Q[:len(self.q), :len(self.q)] *= Y

        # Slack component of Q
        Q[len(self.q):, len(self.q):] = (1 / e) * np.eye(6)

        ##### Determine the equality/inequality constraints #####
        # The equality contraints
        Aeq = np.c_[self.jacobe(self.q), np.eye(6)]
        beq = velocities.reshape((6,))

        # The inequality constraints for joint limit avoidance
        Ain = np.zeros((len(self.q) + 6, len(self.q) + 6))
        bin = np.zeros(len(self.q) + 6)

        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.05

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        # Form the joint limit velocity damper
        Ain[:len(self.q), :len(self.q)], bin[:len(self.q)] = self.joint_velocity_damper(ps, pi, len(self.q))

        ###### TODO: look for collision objects and form velocity damper constraints #####
        for collision in self.collision_obj_list:
            # print(f"collision obj: {collision}")
            # Form the velocity damper inequality contraint for each collision
            # object on the robot to the collision in the scene
            c_Ain, c_bin = self.link_collision_damper(
                collision,
                self.q[:len(self.q)],
                0.3,
                0.05,
                1.0,
                start=self.link_dict["link1"],
                end=self.link_dict["link_eef"],
            )

            # print(f"c_Ain: {np.shape(c_Ain)} | Ain: {np.shape(Ain)}")
            # If there are any parts of the robot within the influence distance
            # to the collision in the scene
            if c_Ain is not None and c_bin is not None:
                c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 5))]

                # print(f"c_Ain (in prob area): {np.shape(c_Ain)} | Ain: {np.shape(Ain)}")
                # Stack the inequality constraints
                Ain = np.r_[Ain, c_Ain]
                bin = np.r_[bin, c_bin]

        # Linear component of objective function: the manipulability Jacobian
        c = np.r_[-self.jacobm(self.q).reshape((len(self.q),)), np.zeros(6)]

        # The lower and upper bounds on the joint velocity and slack variable
        if np.any(self.qdlim):
            lb = -np.r_[self.qdlim[:len(self.q)], 10 * np.ones(6)]
            ub = np.r_[self.qdlim[:len(self.q)], 10 * np.ones(6)]

            # Solve for the joint velocities dq
            qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='daqp')
        else:
            qd = None

        return qd

    def preempt(self, *args: list) -> None:
        """
        Stops any current motion
        """
        # pylint: disable=unused-argument
        if self.executor:
            self.executor.abort()

        self.preempted = True
        self._controller_mode = ControlMode.JOINTS
        self.last_update = 0

    def __vel_move(self, twist_stamped: TwistStamped) -> None:
        target: Twist = twist_stamped.twist

        if twist_stamped.header.frame_id == '':
            twist_stamped.header.frame_id = self.base_link.name

        e_v = np.array([
            target.linear.x,
            target.linear.y,
            target.linear.z,
            target.angular.x,
            target.angular.y,
            target.angular.z
        ])
        
        if np.any(e_v - self.e_v) or self._controller_mode == ControlMode.JOINTS:
            self.e_p = self.fkine(self.q, start=self.base_link, end=self.gripper)

        self.e_v = e_v
        self.e_v_frame = twist_stamped.header.frame_id

        self._controller_mode = ControlMode.CARTESIAN
        self.last_update = self.get_time()

    def get_state(self) -> ManipulatorState:
        """
        Generates a ManipulatorState message for the robot

        :return: ManipulatorState message describing the current state of the robot
        :rtype: ManipulatorState
        """
        jacob0 = self.jacob0(self.q, end=self.gripper)
        
        ## end-effector position
        ee_pose = self.ets(start=self.base_link, end=self.gripper).eval(self.q)
        header = Header()
        header.frame_id = self.base_link.name
        header.stamp = self.get_stamp()

        pose_stamped = PoseStamped()
        pose_stamped.header = header

        translation = ee_pose[:3, 3]    
        pose_stamped.pose.position.x = translation[0]
        pose_stamped.pose.position.y = translation[1]
        pose_stamped.pose.position.z = translation[2]

        rotation = ee_pose[:3, :3]
        ee_rot = sm.UnitQuaternion(rotation)

        pose_stamped.pose.orientation.w = ee_rot.A[0]
        pose_stamped.pose.orientation.x = ee_rot.A[1]
        pose_stamped.pose.orientation.y = ee_rot.A[2]
        pose_stamped.pose.orientation.z = ee_rot.A[3]

        state = ManipulatorState()
        state.ee_pose = pose_stamped

        # end-effector velocity
        T = jacob0 @ self.qd

        twist_stamped = TwistStamped()
        twist_stamped.header = header
        twist_stamped.twist.linear.x = T[0]
        twist_stamped.twist.linear.y = T[1]
        twist_stamped.twist.linear.z = T[2]
        twist_stamped.twist.angular.x = T[3]
        twist_stamped.twist.angular.x = T[4]
        twist_stamped.twist.angular.x = T[5]

        state.ee_velocity = twist_stamped
        
        # joints
        if self.joint_states:
            state.joint_poses = np.array(self.joint_states.position)[self.joint_indexes]
            state.joint_velocities = np.array(self.joint_states.velocity)[self.joint_indexes]
            state.joint_torques = np.array(self.joint_states.effort)[self.joint_indexes]
        
        else:
            state.joint_poses = list(self.q)
            state.joint_velocities = list(self.qd)
            state.joint_torques = np.zeros(self.n, dtype=np.float64).tolist()
        
        return state

    def test_guards(
        self,
        guards: Guards,
        start_time: float) -> int:

        triggered = 0

        if (guards.enabled & guards.GUARD_DURATION) == guards.GUARD_DURATION:
            triggered |= guards.GUARD_DURATION if self.get_time() - start_time > guards.duration else 0

        if (guards.enabled & guards.GUARD_EFFORT) == guards.GUARD_EFFORT:
            eActual = np.fabs(np.array([
                self.state.ee_wrench.wrench.force.x,
                self.state.ee_wrench.wrench.force.y,
                self.state.ee_wrench.wrench.force.z,
                self.state.ee_wrench.wrench.torque.x,
                self.state.ee_wrench.wrench.torque.y,
                self.state.ee_wrench.wrench.torque.z,
            ]))

            eThreshold = np.array([
                guards.effort.force.x,
                guards.effort.force.y,
                guards.effort.force.z,
                guards.effort.torque.x,
                guards.effort.torque.y,
                guards.effort.torque.z,
            ])

            triggered |= guards.GUARD_EFFORT if np.any(eActual > eThreshold) else 0
            
        return triggered

    def publish(self):
        self.joint_publisher.publish(Float64MultiArray(data=self.qd))

    def check_singularity(self, q=None) -> bool:
        """
        Checks the manipulability as a scalar manipulability index
        for the robot at the joint configuration to indicate singularity approach. 
        - It indicates dexterity (how well conditioned the robot is for motion)
        - Value approaches 0 if robot is at singularity
        - Returns True if close to singularity (based on threshold) or False otherwise
        - See rtb.robots.Robot.py for details

        :param q: The robot state to check for manipulability.
        :type q: numpy array of joints (float)
        :return: True (if within singularity) or False (otherwise)
        :rtype: bool
        """
        # Get the robot state manipulability
        self.manip_scalar = self.manipulability(q)

        # Debugging
        # rospy.loginfo(f"Manipulability: {manip_scalar} | --> 0 is singularity")

        if (np.fabs(self.manip_scalar) <= self.singularity_thresh and self.preempted == False):
            self.singularity_approached = True
            self.logger(f"Preempted due to singularity {self.manip_scalar}", 'warn')
            return True
        else:
            self.singularity_approached = False
            return False
        
    def transform(self, stamped_message, target_frame_id):
        T = self.tf_buffer.lookup_transform(
        target_frame_id, 
        stamped_message.header.frame_id,
        self.get_time(False)
        )
      
        return stamped_message

    def get_time(self, as_float=True):
        if hasattr(self.nh, 'get_clock'):
            if as_float:
                return self.nh.get_clock().now().nanoseconds / 1e9
            return self.nh.get_clock().now()
        else:
            if as_float:
                return self.nh.get_time()
            return self.nh.Time().now()

    def get_stamp(self):
        if hasattr(self.nh, 'get_clock'):
            return self.nh.get_clock().now().to_msg()
        return self.nh.Time.now()

    def get_parameter(self, param_name, default_value=None):
        if hasattr(self.nh, 'get_parameter'):
            if not self.nh.has_parameter(param_name):
                self.nh.declare_parameter(param_name, default_value)
            return self.nh.get_parameter(param_name).value
        return self.nh.get_param(param_name, default_value)

    def __load_named_pose_config(self):
        """[summary]
        """
        self.named_poses = {}
        for config_name in self.custom_configs:
            try:
                config = yaml.load(open(config_name))
                if config and 'named_poses' in config:
                    self.named_poses.update(config['named_poses'])
            except IOError:
                self.logger(
                    'Unable to locate configuration file: {}'.format(config_name), 'warn'
                )

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
                current = yaml.load(handle.read(), Loader=yaml.SafeLoader)

                if current:
                    config = current

        except IOError:
            pass

        config.update({key: value})

        with open(self.config_path, 'w') as handle:
            handle.write(yaml.dump(config))

    # --------------------------------------------------------------------- #
    # --------- Main Execution Method for ARMer --------------------------- #
    # --------------------------------------------------------------------- #
    def step(self, dt: float = 0.01) -> None:  # pylint: disable=unused-argument
        """
        Updates the robot joints (robot.q) used in computing kinematics
        :param dt: the delta time since the last update, defaults to 0.01 (must be in seconds)
        :type dt: float         
        """
        if self.readonly:
            return

        current_time = self.get_time()
        self.state = self.get_state()

        # PREEMPT motion on any detected state errors or singularity approach
        if self.state.errors != 0 or self.check_singularity(self.q):
            self.preempt()

        ## -- DIRECT CARTESIAN CONTROL -- ##
        if self._controller_mode == ControlMode.CARTESIAN:
            # Handle communication delays and error handling on cartesian control
            if current_time - self.last_update > 0.1:
                self.e_v *= 0.9 if np.sum(np.absolute(self.e_v)
                                          ) >= 0.0001 else 0

                if np.all(self.e_v == 0):
                    self._controller_mode = ControlMode.JOINTS

            try:
                # DEBUGGING
                # print(f"base_link name: {self.base_link.name}")
                # print(f"e_v_frame name: {self.e_v_frame}")

                # Get the transform from the defined base link to the target (as requested)
                # NOTE: 
                # - this lookup returns a TransformStamped type
                # - the time is 0 to get the latest
                tfs = self.tf_buffer.lookup_transform(
                    target_frame=self.base_link.name,
                    source_frame=self.e_v_frame,
                    time=rclpy.time.Time()
                )

                # Convert tfs to an SE3
                U = sm.UnitQuaternion([
                    tfs.transform.rotation.w,
                    tfs.transform.rotation.x,
                    tfs.transform.rotation.y,
                    tfs.transform.rotation.z,
                ], norm=True, check=False).SE3()
                
                e_v = np.concatenate((
                (U.A @ np.concatenate((self.e_v[:3], [1]), axis=0))[:3],
                (U.A @ np.concatenate((self.e_v[3:], [1]), axis=0))[:3]
                ), axis=0)
                
                # -- Calculate error in base frame -- #
                # Get the expected position/rotation
                p = self.e_p.A[:3, 3] + e_v[:3] * dt                     
                Rq = sm.UnitQuaternion.RPY(e_v[3:] * dt) * sm.UnitQuaternion(self.e_p.R)
                
                # Convert to an expected pose for comparison with the actual pose
                T = sm.SE3.Rt(sm.SO3(Rq.R), p, check=False) 
                Tactual = self.fkine(self.q, start=self.base_link, end=self.gripper)
                
                # Calculate positional/rotational error with actual
                e_rot = (sm.SO3(T.R @ np.linalg.pinv(Tactual.R), check=False).rpy() + np.pi) % (2*np.pi) - np.pi
                error = np.concatenate((p - Tactual.t, e_rot), axis=0)
                
                # Apply error
                e_v = e_v + error
                # Updated expected pose
                self.e_p = T
                # Apply as joint velocities
                self.j_v = np.linalg.pinv(self.jacob0(self.q, end=self.gripper)) @ e_v
              
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
              self.logger('No valid transform found between {} and {}'.format(self.base_link.name, self.e_v_frame), 'warn')
              self.preempt()

        ## -- TRAJECTORY-BASED EXECUTION UPDATE -- ##
        if self.executor:
            # self.logger(f"Requested traj")
            self.j_v = self.executor.step(dt)  
        else:
            # Needed for preempting joint velocity control
            if any(self.j_v) and current_time - self.last_update > 0.1:
                self.j_v *= 0.9 if np.sum(np.absolute(self.j_v)) >= 0.0001 else 0
            
        ## -- FINAL ROBOT JOINT VELOCITY UPDATE -- ##
        # NOTE: check for joint array length miss-match and handle
        if len(self.j_v) != len(self.qd):
            self.logger(f"Incorrect velocity vector size {len(self.j_v)} | requires: {len(self.qd)}", 'warn')
            self.preempt()
        else:
            self.qd = self.j_v

        # Update for next cycle
        self.last_tick = current_time
        self.state_publisher.publish(self.state)
        self.event.set()