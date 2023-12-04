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

from typing import List, Any
from threading import Lock, Event
from armer.models.URDFRobot import URDFRobot
from armer.utils import mjtg, trapezoidal, ikine
from armer.errors import ArmerError
from armer.trajectory import TrajectoryExecutor
from armer.timer import Timer
from rclpy.callback_groups import ReentrantCallbackGroup

# pylint: disable=too-many-instance-attributes

# ROS Message Imports
from armer_msgs.msg import ManipulatorState, JointVelocity, ServoStamped, Guards
from armer_msgs.action import GuardedVelocity, Home, MoveToJointPose, MoveToNamedPose, MoveToPose
from armer_msgs.srv import GetNamedPoses, AddNamedPose, RemoveNamedPose, AddNamedPoseConfig, RemoveNamedPoseConfig, GetNamedPoseConfigs 
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped, Pose
from std_msgs.msg import Header, Float64MultiArray, Bool
from sensor_msgs.msg import JointState

# TESTING NEW IMPORTS FOR DYNAMIC OBJECT MARKER DISPLAY (RVIZ)
from visualization_msgs.msg import Marker, MarkerArray

# Collision Checking Imports
from sklearn.neighbors import KDTree
from armer.cython import collision_handler
from dataclasses import dataclass
import qpsolvers as qp
import spatialgeometry as sg
import math

class ControlMode:
   JOINTS=1
   CARTESIAN=2

class ControllerType:
    JOINT_GROUP_VEL=0
    JOINT_TRAJECTORY=1

# Class of dynamic objects (input at runtime)
@dataclass
class DynamicCollisionObj:
    shape: sg.Shape
    key: str = ''
    id: int = 0
    pose: Pose = Pose()
    is_added: bool = False

class ROSRobot(URDFRobot):
    """
    The ROSRobot class wraps the URDFRobot implementing basic ROS functionality
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
                 qlim_min=None,
                 qlim_max=None,
                 qdlim=None,
                 * args,
                 **kwargs):  # pylint: disable=unused-argument
        
        super().__init__(nh, *args, **kwargs)
        
        # Setup the name of the robot
        self.name = name if name else self.name
        # Configure action servers with their required callback group
        self.action_cb_group = action_cb_group if action_cb_group else ReentrantCallbackGroup()
                
        # Update with ustom qlim (joint limits) if specified in robot config
        if qlim_min and qlim_max:
            self.qlim = np.array([qlim_min, qlim_max])
            self.logger(f"Updating Custom qlim: {self.qlim}")

        if qdlim:
            self.qdlim = np.array(qdlim)
            self.logger(f"Updating Custom qdlim: {self.qdlim}")
        else:
            self.qdlim = None

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
            os.getenv('HOME', '/home'),
            '.ros/configs/system_named_poses.yaml'
        )

        # Configure the gripper name
        if not hasattr(self, 'gripper'):
            self.gripper = self.grippers[0].name if len(self.grippers) > 0 else 'tool0'
          
        # Sort links by parents starting from gripper
        self.sorted_links=[]

        # Check if the existing gripper name exists (Error handling) otherwise default to top of dict stack
        if self.gripper not in self.link_dict.keys():
            default_top_link_name = sorted(self.link_dict.keys())[-1]
            self.logger(f"Configured gripper name {self.gripper} not in link tree -> defaulting to top of stack: {default_top_link_name}")
            self.gripper = default_top_link_name

        link=self.link_dict[self.gripper]   
        while link is not None:
            self.sorted_links.append(link)
            link=link.parent
        self.sorted_links.reverse()

        # --- Collision Checking Setup Section --- #
        # Loops through links (as read in through URDF parser)
        # Extracts each link's list of collision shapes (of type sg.Shape). TODO: add a validity check on type here
        # Updates a dictionary (key as link name) of collision shape lists per link
        # Create a new dictionary of link lookup to ignore overlapping joint/collision objects
        # NOTE: the theory here is to keep a record of all expected collisions (overlaps) per link so that
        #       the main self collision check can ignore these cases. 
        self.overlapped_link_dict = dict()
        self.collision_dict = dict()
        # This is a list of collision objects that is used by NEO
        # NOTE: this may not be explictly needed, as a list form (flattened) of the self.collision_dict.values() 
        #       contains the same information
        self.collision_obj_list = list()
        # Define a list of dynamic objects (to be added in at runtime)
        # NOTE: this is useful to include collision shapes (i.e., cylinders, spheres, cuboids) in your scene for collision checking
        self.dynamic_collision_dict = dict()
        self.dynamic_collision_removal_dict = dict()
        self.collision_approached = False
        # Check for external links (from robot tree)
        # This is required to add any other links (not specifically part of the robot tree) 
        # to our collision dictionary for checking
        for link in self.links:
            # print(f"links: {link.name}")
            if link.name in self.collision_dict.keys():
                continue

            # print(f"adding link: {link.name} which is type: {type(link)} with collision data: {link.collision.data}")
            self.collision_dict[link.name] = link.collision.data if link.collision.data else []
            [self.collision_obj_list.append(data) for data in link.collision.data]
        
        # Handle collision link window slicing
        # Window slicing is a way to define which links need to be tracked for collisions (for efficiency)
        # Checks error of input from configuration file (note that default is current sorted links)
        self.collision_sliced_links = self.sorted_links
        self.update_link_collision_window()
        # Initialise a 'ghost' robot instance for trajectory collision checking prior to execution
        # NOTE: Used to visualise the trajectory as a simple marker list in RVIZ for debugging
        # NOTE: there was an issue identified around using the latest franka-emika panda description
        # self.logger(f"urdf filepath: {self.urdf_filepath}")
        self.robot_ghost = URDFRobot(
            nh=nh,
            wait_for_description=False, 
            gripper=self.gripper,
            urdf_file=self.urdf_filepath,
            collision_check_start_link=self.collision_sliced_links[-1].name, 
            collision_check_stop_link=self.collision_sliced_links[0].name
        )
        if self.robot_ghost.is_valid():
            self.logger(f"Robot Ghost Configured and Ready")
        else:
            self.logger(f"Robot Ghost Invalid -> Cannot Conduct Trajectory Collision Checking", 'warn')
        
        # Collision Safe State Window
        # NOTE: the size of the window (default of 200) dictates how far back we move to get out
        # of a collision situation, based on previously executed joint states that were safe
        self.q_safe_window = np.zeros([200,len(self.q)])
        self.q_safe_window_p = 0
        # --- Collision Checking Setup Section END --- #
        
        self.joint_indexes = []
        self.joint_names = list(map(lambda link: link._joint_name, filter(lambda link: link.isjoint, self.sorted_links)))
        
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
        self.traj_generator = trapezoidal
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
            self.remove_named_pose_srv = self.nh.create_service(
                RemoveNamedPose, 
                '{}/remove_named_pose'.format(self.name.lower()), 
                self.remove_named_pose_cb
            )
            self.add_named_pose_config_srv = self.nh.create_service(
                AddNamedPoseConfig, 
                '{}/add_named_pose_config'.format(self.name.lower()), 
                self.add_named_pose_config_cb
            )
            self.remove_named_pose_config_srv = self.nh.create_service(
                RemoveNamedPoseConfig, 
                '{}/remove_named_pose_config'.format(self.name.lower()), 
                self.remove_named_pose_config_cb
            )
            self.get_named_pose_config_srv = self.nh.create_service(
                GetNamedPoseConfigs, 
                '{}/get_named_pose_configs'.format(self.name.lower()), 
                self.get_named_pose_configs_cb
            )

        # TODO: add these services when ready
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
            # Error handling on missing pose name
            if not request.pose_name in self.named_poses:
                self.logger(f"Unknown named pose: {request.pose_name}", 'warn')
                result = False
                goal_handle.abort()

            else:
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

    def remove_named_pose_cb(self, request, response):
        """
        ROS Service callback:
        Adds the current arm pose as a named pose and saves it to the host config

        :param req: The name of the pose as well as whether to overwrite if the pose already exists
        :type req: AddNamedPoseRequest
        :return: True if the named pose was written successfully otherwise false
        :rtype: AddNamedPoseResponse
        """
        if request.pose_name not in self.named_poses and not request.overwrite:
            self.logger('Named pose does not exists.', 'warn')
            response.success=False
        else:
            del self.named_poses[request.pose_name]
            self.__write_config('named_poses', self.named_poses)
            response.success=True

        return response

    def add_named_pose_config_cb(self, request, response):
        """[summary]

        :param request: [description]
        :type request: AddNamedPoseConfigRequest
        :return: [description]
        :rtype: AddNamedPoseConfigResponse
        """
        self.custom_configs.append(request.config_path)
        self.__load_named_pose_config()
        
        print(f"response attr: {response.__dir__()}")
        return True

    def remove_named_pose_config_cb(self, request, response):
        """[summary]

        :param request: [description]
        :type request: AddNamedPoseRequest
        :return: [description]
        :rtype: [type]
        """
        if request.config_path in self.custom_configs:
            self.custom_configs.remove(request.config_path)
            self.__load_named_pose_config()

        print(f"response attr: {response.__dir__()}")
        return True

    def get_named_pose_configs_cb(self, request, response):
        """[summary]

        :param request: [description]
        :type request: GetNamedPoseConfigsRequest
        :return: [description]
        :rtype: GetNamedPoseConfigsResponse
        """
        response.configs = str(self.custom_configs)
        return response
    
    # --------------------------------------------------------------------- #
    # --------- Collision and Singularity Checking Methods ---------------- #
    # --------------------------------------------------------------------- #
    def add_collision_obj(self, obj):
        """
        Simple mechanism for adding a shape object to the collision list
        NOTE: only for debugging purposes
        """
        self.collision_obj_list.append(obj)

    def closest_dist_query_shape_based(self, sliced_link_name, target_links):
        """
        This method uses the closest point method of a Shape object to extract translation to a link.
        NOTE: only a single shape is used per link (defaulting to the first one). This is needed to
        keep the speed of this process as fast as possible. This may not be the best method if the 
        shape being used is not 'representative' of the entire link.
        """
        translation_dict = {
            link: self.collision_dict[sliced_link_name][0].closest_point(self.collision_dict[link][0])[2]
            for link in target_links 
            if self.collision_dict[link] != [] and link not in self.overlapped_link_dict[sliced_link_name]
        }
        return translation_dict
    
    def closest_dist_query_pose_based(self, sliced_link_name, magnitude_thresh: float = 0.4, refine: bool = False):
        """
        This method uses the built-in recursive search (ets) for links within the robot tree
        to quickly get the translation from a specified sliced link. Note that this method
        gets the translation to the link's origin, which may not be as representative as possibe with
        respect to the surface of the link.
        NOTE: an additional external dictionary is needed for dynamically added shapes, or shapes/links not
        within the robot tree (fails at the ets method)
        NOTE: currently the fastest method as of 2023-12-1
        """
        col_dict_cp = self.collision_dict.copy()
        # If asked to refine, then base the link dictionary creation on the magnitude threshold value (m)
        # NOTE: refining can lead to slow down on large number of multiple shapes
        if refine:
            translation_dict = {
                link: self.ets(start=sliced_link_name, end=link).eval(self.q)[:3, 3]
                for link in col_dict_cp.keys()
                if link in self.link_dict.keys() 
                    and col_dict_cp[link] != [] 
                    and link not in self.overlapped_link_dict[sliced_link_name]
                    and link != sliced_link_name
                    and np.linalg.norm(
                        self.ets(start=sliced_link_name, end=link).eval(self.q)[:3, 3]
                    ) < magnitude_thresh
            }

            # This is the external objects translation from the base_link
            external_dict = {
                link: col_dict_cp[link][0].T[:3,3]
                for link in col_dict_cp.keys()
                if link not in self.link_dict.keys()
                    and col_dict_cp[link] != []
                    and link not in self.overlapped_link_dict[sliced_link_name]
                    and link != sliced_link_name
                    and np.linalg.norm(
                        math.dist(
                            self.ets(start=self.base_link.name, end=sliced_link_name).eval(self.q)[:3, 3],
                            col_dict_cp[link][0].T[:3,3]
                        )
                    ) < magnitude_thresh
            }
        else:
            translation_dict = {
                link: self.ets(start=sliced_link_name, end=link).eval(self.q)[:3, 3]
                for link in col_dict_cp.keys()
                if link in self.link_dict.keys() 
                    and col_dict_cp[link] != [] 
                    and link not in self.overlapped_link_dict[sliced_link_name]
                    and link != sliced_link_name
            }

            # This is the external objects translation from the base_link
            external_dict = {
                link: col_dict_cp[link][0].T[:3,3]
                for link in col_dict_cp.keys()
                if link not in self.link_dict.keys()
                    and col_dict_cp[link] != []
                    and link not in self.overlapped_link_dict[sliced_link_name]
                    and link != sliced_link_name
            }

        translation_dict.update(external_dict)
        return translation_dict
    
    def collision_marker_debugger(self, sliced_link_names: list = [], check_link_names: list = []):
        """
        A simple debugging method to output to RVIZ the current link shapes being checked
        """
        marker_array = []
        captured = []
        counter = 0
        for links in check_link_names:
            # These are the links associated with the respective idx sliced link
            for link in links:
                # Check if we have already created a marker
                if link in captured:
                    continue

                # Get all the shape objects and create a marker for each
                for shape in self.collision_dict[link]:
                    # Default setup of marker header
                    marker = Marker()

                    # NOTE: this is currently an assumption as dynamic objects as easily added with respect to base link
                    # TODO: add with respect to any frame would be good
                    # NOTE: mesh objects not working (need relative pathing, not absolute)
                    if link not in self.link_dict.keys():
                        marker.header.frame_id = self.base_link.name
                    else:
                        marker.header.frame_id = link

                    marker.header.stamp = rospy.Time.now()
                    if shape.stype == 'sphere':
                        marker.type = 2
                        # Expects diameter (m)
                        marker.scale.x = shape.radius * 2
                        marker.scale.y = shape.radius * 2
                        marker.scale.z = shape.radius * 2
                    elif shape.stype == 'cylinder':
                        marker.type = 3

                        # Expects diameter (m)
                        marker.scale.x = shape.radius * 2
                        marker.scale.y = shape.radius * 2
                        marker.scale.z = shape.length
                    elif shape.stype == 'cuboid':
                        marker.type = 1

                        # Expects diameter (m)
                        marker.scale.x = shape.scale[0]
                        marker.scale.y = shape.scale[1]
                        marker.scale.z = shape.scale[2]
                    else:
                        break
                
                    marker.id = counter
                    pose_se3 = sm.SE3(shape.T[:3, 3])
                    marker.pose.position = Point(*pose_se3.t)

                    if link in sliced_link_names:
                        marker.color.r = 0.5
                        marker.color.g = 0.5
                    else:
                        marker.color.r = 0
                        marker.color.g = 1
                    
                    marker.color.b = 0
                    marker.color.a = 0.25
                    counter+=1

                    marker_array.append(marker)
            
                captured.append(link)

        # Publish array of markers
        self.collision_debug_publisher.publish(marker_array)
    
    def query_target_link_check(self, sliced_link_name: str = "", magnitude_thresh: float = 0.2):
        """
        This method is intended to run per cycle of operation and is expected to find changes to distances
        based on the original main dictionary (as created by creation_of_pose_link_distances)
        """
        col_dict_cp = self.collision_dict.copy()
        target_list = [
            link
            for link in col_dict_cp.keys()
            if link in self.link_dict.keys() 
                and col_dict_cp[link] != [] 
                and link not in self.overlapped_link_dict[sliced_link_name]
                and link != sliced_link_name
                and np.linalg.norm(
                    self.ets(start=sliced_link_name, end=link).eval(self.q)[:3, 3]
                ) < magnitude_thresh
        ]

        external_list =[
            link
            for link in col_dict_cp.keys()
            if link not in self.link_dict.keys()
                and col_dict_cp[link] != []
                and link not in self.overlapped_link_dict[sliced_link_name]
                and link != sliced_link_name
                and np.linalg.norm(
                    math.dist(
                        self.ets(start=self.base_link.name, end=sliced_link_name).eval(self.q)[:3, 3],
                        col_dict_cp[link][0].T[:3,3]
                    )
                ) < magnitude_thresh
        ]

        return target_list + external_list

    def query_kd_nn_collision_tree(self, sliced_links: list = [], dim: int = 5, debug: bool = False) -> list:
        """
        Given a list of links (sliced), this method returns nearest neighbor links for collision checking
        Aims to improve efficiency by identifying dim closest objects for collision checking per link
        """
        # Early termination
        if sliced_links == None or sliced_links == []:
            self.logger(f"target links: {sliced_links} is not valid. Exiting...", 'error')
            return []

        # print(f"cylinder link check: {self.link_dict['cylinder_link']}")
        # Iterate through each sliced link (target link) 
        # For the current sliced link; find the closest point between one of the link's shapes
        # NOTE: each link can have multiple shapes, but in the first instance, we take only one shape per link to 
        #       understand the distance to then calculate the target links via the KDTree
        check_links = []
        for sliced_link in sliced_links:
            # Early termination on error
            if sliced_link.name not in self.link_dict.keys():
                self.logger(f"Given sliced link: {sliced_link.name} is not valid. Skipping...", 'error')
                continue

            # Testing refinement using link poses (Individual method needed for shape-based version)
            # start = timeit.default_timer()
            # target_link_list = self.query_target_link_check(sliced_link_name=sliced_link.name, magnitude_tresh=0.4)
            # end = timeit.default_timer()
            # print(f"[Link Pose Based] Extraction of Target Links for Surface Check: {1/(end-start)} hz")

            # Initial approach to get closest link (based on link origin, not surface)
            # NOTE: as it is based on origin, the size/shape of collision object matters
            # NOTE: fastest method as of 2023-12-1
            start = timeit.default_timer()
            translation_dict = self.closest_dist_query_pose_based(sliced_link_name=sliced_link.name)
            end = timeit.default_timer()
            # print(f"[OLD] Get distances to links from target: {1/(end-start)} hz")
         
            tree = KDTree(data=list(translation_dict.copy().values()))
            target_position = self.ets(start=self.base_link, end=sliced_link).eval(self.q)[:3, 3]
            # Test query of nearest neighbors for a specific shape (3D) as origin (given tree is from source)
            dist, ind = tree.query(X=[target_position], k=len(translation_dict.keys()) if len(translation_dict.keys()) < dim else dim, dualtree=True)
            # print(f"dist: {dist} | links: {[list(translation_dict.keys())[i] for i in ind[0]]} | ind[0]: {ind[0]}")

            check_links.append([list(translation_dict.keys())[i] for i in ind[0]])
       
        if debug:
            self.collision_marker_debugger(
                sliced_link_names=[link.name for link in sliced_links],
                check_link_names=check_links
            )

        return check_links

    def trajectory_collision_checker(self, traj) -> bool:
        """
        - Checks against a given trajectory (per state) if a collision is found
        - Outputs a debugging marker trail of path for visualisation
        
        Returns True valid (no collisions), otherwise False if found
        """
        # Attempt to slice the trajectory into bit-size chuncks to speed up collision check
        # NOTE: doesn't need to check every state in trajectory, as spatially stepping will find links in collision
        # NOTE: the smaller the step thresh is, the less points used for checking (resulting in faster execution)
        # TODO: check this assumption holds true
        step_thresh = 10
        step_value = int(len(traj.s) / step_thresh) if int(len(traj.s) / step_thresh) > 0 else 1
        
        # Create marker array for representing trajectory
        marker_traj = Marker()
        marker_traj.header.frame_id = self.base_link.name
        marker_traj.header.stamp = rospy.Time.now()
        marker_traj.action = Marker.ADD
        marker_traj.pose.orientation.w = 1
        marker_traj.id = 0
        marker_traj.type = Marker.LINE_STRIP
        marker_traj.scale.x = 0.01
        # Default to Green Markers
        marker_traj.color.g = 1.0
        marker_traj.color.a = 1.0

        # Initial empty publish
        marker_traj.points = []
        self.display_traj_publisher.publish(marker_traj)

        self.logger(f"Traj len: {len(traj.s)} | with step value: {step_value}")
        go = True
        with Timer("Full Trajectory Collision Check", enabled=True):
            for idx in range(0,len(traj.s),step_value):
                
                # Calculate end-effector pose and extract translation component
                pose = self.ets(start=self.base_link, end=self.gripper).eval(traj.s[idx])
                extracted_t = pose[:3, 3]

                # Update marker trajectory for visual representation
                p = Point()
                p.x = extracted_t[0]
                p.y = extracted_t[1]
                p.z = extracted_t[2]
                marker_traj.points.append(p)

                print(f"extracted translation: {extracted_t}")
                if self.check_collision_per_state(q=traj.s[idx]) == False:
                    # Terminate on collision check failure
                    # TODO: add red component to trail to signify collision
                    # NOTE: currently terminates on collision for speed and efficiency
                    #       could continue (regardless) to show 'full' trajectory with collision
                    #       component highlighted? Note, that speed is a function of 
                    #       step_thresh (currently yields approx. 30Hz on calculation 
                    #       for a 500 sample size traj at a step_tresh of 10)
                    go=False
                    break

        # Publish marker (regardless of failure case for visual identification)
        self.display_traj_publisher.publish(marker_traj)
        # Output go based on passing collision check
        return go
    
    def get_link_collision_dict(self) -> dict():
        """
        Returns a dictionary of all associated links (names) which lists their respective collision data
        To be used by high-level armer class for collision handling
        """
        return self.collision_dict
    
    def check_collision_per_state(self, q: list() = []) -> bool():
        """
        Given a robot state (q) this method checks if the links (of a ghost robot) will result in a collision
        If a collision is found, then the output is True, else False
        """
        with Timer(name="setup backend and state", enabled=False):    
            env = self.robot_ghost._get_graphical_backend()
            self.robot_ghost.q = q
            env.launch(headless=True)
            env.add(self.robot_ghost, readonly=True)
                
            go_signal = True

        with Timer(f"Get collision per state:", enabled=False):
            # NOTE: use current (non-ghost) robot's link names to extract ghost robot's link from dictionary
            for link in self.collision_dict.keys():
                # Get the link to check against from the ghost robot's link dictionary
                # Output as a list of names
                links_in_collision = self.get_links_in_collision(
                    target_link=link, 
                    check_list=self.robot_ghost.link_dict[link].collision.data if link in self.robot_ghost.link_dict.keys() else self.collision_dict[link], 
                    ignore_list=self.overlapped_link_dict[link] if link in self.overlapped_link_dict.keys() else [],
                    link_list=self.robot_ghost.links,
                    output_name_list=True,
                    skip=True)
            
                # print(f"Checking [{link}] -> links in collision: {links_in_collision}")
                
                if len(links_in_collision) > 0:
                    self.logger(f"Collision at state {self.robot_ghost.q} in trajectory between -> [{link}] and {[link_n for link_n in links_in_collision]}", 'error')
                    go_signal = False
                    break

            return go_signal 
    
    def update_link_collision_window(self):
        """
        This method updates a sliced list of links (member variable)
        as determined by the class method variables:
            collision_check_start_link
            collision_check_stop_link
        """
        with Timer("Link Slicing Check", enabled=False):
            # Prepare sliced link based on a defined stop link 
            # TODO: this could be update-able for interesting collision checks based on runtime requirements
            # NOTE: the assumption here is that each link is unique (which is handled low level by rtb) so we take the first element if found
            # NOTE: sorted links is from base link upwards to end-effector. We want to slice from stop link to start in rising index order
            col_start_link_idx = [i for i, link in enumerate(self.sorted_links) if link.name == self.collision_check_start_link]
            col_stop_link_idx = [i for i, link in enumerate(self.sorted_links) if link.name == self.collision_check_stop_link]
            # print(f"start_idx: {col_start_link_idx} | stop_idx: {col_stop_link_idx}")

            # NOTE: slice indexes are lists, so confirm data inside
            if len(col_start_link_idx) > 0 and len(col_stop_link_idx) > 0:
                start_idx = col_start_link_idx[0]
                end_idx = col_stop_link_idx[0]

                # Terminate early on invalid indexes
                if start_idx < end_idx or start_idx > len(self.sorted_links):
                    self.logger(f"Start and End idx are incompatible, defaulting to full link list", 'warn')
                    return 

                # Handle end point
                if start_idx == len(self.sorted_links):
                    self.collision_sliced_links = self.sorted_links[end_idx:None]
                else:
                    self.collision_sliced_links = self.sorted_links[end_idx:start_idx + 1]

                # Reverse order for sorting from start to end
                self.collision_sliced_links.reverse()    

                self.logger(f"Collision Link Window Set: {[link.name for link in self.collision_sliced_links]}")
            else:
                # Defaul to the current list of sorted links (full)
                self.collision_sliced_links = self.sorted_links
                self.collision_sliced_links.reverse()

    def characterise_collision_overlaps(self) -> bool:
        """
        Characterises the existing robot tree and tracks overlapped links in collision handling
        NOTE: needed to do collision checking, when joints are typically (neighboring) overlapped
        NOTE: this is quite an intensive run at the moment, 
            however, it is only expected to be run in single intervals (not continuous)
            [2023-10-27] approx. time frequency is 1hz (Panda simulated)
            [2023-10-31] approx. time frequency is 40Hz and 21Hz (UR10 and Panda simulated with better method, respectively)
        """
        # Running timer to get frequency of run. Set enabled to True for debugging output to stdout
        with Timer(name="Characterise Collision Overlaps", enabled=True):
            # Error handling on gripper name
            if self.gripper == None or self.gripper == "":
                self.logger(f"Characterise Collision Overlaps -> gripper name is invalid: {self.gripper}", 'error')
                return False 
            
            # Error handling on empty lick dictionary (should never happen but just in case)
            if self.link_dict == dict() or self.link_dict == None:
                self.logger(f"Characterise Collision Overlaps -> link dictionary is invalid: {self.link_dict}", 'error')
                return False
            
            # Error handling on collision object dict
            if self.collision_dict == dict() or self.collision_dict == None:
                self.logger(f"Characterise Collision Overlaps -> collision dictionary is invalid: [{self.collision_dict}]", 'error')
                return False
            
            # Alternative Method (METHOD 2) that is getting the list in a faster iterative method
            # NOTE: this has to course through ALL links in space (self.links encapsulates all links that are not the gripper)
            self.overlapped_link_dict = dict([
                (link, self.get_links_in_collision(
                    target_link=link, 
                    check_list=self.collision_dict[link], 
                    ignore_list=[],
                    link_list=self.links,
                    output_name_list=True)
                )
                for link in self.collision_dict.keys()])
            
            # NOTE: secondary run to get gripper links as well
            gripper_dict = dict([
                (link.name, self.get_links_in_collision(
                    target_link=link.name, 
                    check_list=self.collision_dict[link.name], 
                    ignore_list=[],
                    output_name_list=True)
                )
                for link in reversed(self.grippers)])
            
            self.overlapped_link_dict.update(gripper_dict)

            # using json.dumps() to Pretty Print O(n) time complexity
            self.logger(f"Characterise Collision Overlaps per link: {json.dumps(self.overlapped_link_dict, indent=4)}")

        # Reached end in success
        return True
    
    def get_links_in_collision(self, target_link: str, 
                               ignore_list: list = [], 
                               check_list: list = [], 
                               link_list: list = [], 
                               output_name_list: bool = False,
                               skip: bool = True):
        """
        An alternative method that returns a list of links in collision with target link.
        NOTE: ignore list used to ignore known overlapped collisions (i.e., neighboring link collisions)
        NOTE: check_list is a list of Shape objects to check against.
        """
        with Timer("NEW Get Link Collision", enabled=False):
            # rospy.loginfo(f"Target link requested is: {target_link}")
            if link_list == []:
                link_list = self.sorted_links
            
            # Handle invalid link name input
            if target_link == '' or target_link == None or not isinstance(target_link, str):
                self.logger(f"Self Collision Check -> Link name [{target_link}] is invalid.", 'warn')
                return []
            
            # Handle check list empty scenario
            if check_list == []:
                # print(f"Check list is empty so terminate.")
                return []

            # DEBUGGING
            # rospy.loginfo(f"{target_link} has the following collision objects: {check_list}")
            
            # NOTE iterates over all configured links and compares against provided list of check shapes 
            # NOTE: the less objects in a check list, the better
            #       this is to handle cases (like with the panda) that has multiple shapes per link defining its collision geometry
            #       any custom descriptions should aim to limit the geometry per link as robot geometry is controlled by the vendor
            # NOTE: ignore list is initiased at start up and is meant to handle cases where a mounted table (in collision with the base) is ignored
            #       i.e., does not throw a collision for base_link in collision with table (as it is to be ignored) but will trigger for end-effector link
            check_dict = dict([(link.name, link) \
                for obj in check_list \
                for link in reversed(link_list) \
                if (link.name not in ignore_list) and (link.name != target_link) and (link.iscollided(obj, skip=skip))
            ])
            
            # print(f"links: {[link.name for link in self.links]}")
            # print(f"Collision Keys: {list(check_dict.keys())}") if len(check_dict.keys()) > 0 else None   
            # print(f"Collision Values: {list(check_dict.values())}")    

            # Output list of collisions or name of links based on input bool
            if output_name_list:
                return list(check_dict.keys())
            else:
                return list(check_dict.values())
    
    def check_link_collision(self, target_link: str, sliced_links: list = [], ignore_list: list = [], check_list: list = []):
        """
        This method is similar to roboticstoolbox.robot.Robot.iscollided
        NOTE: ignore list used to ignore known overlapped collisions (i.e., neighboring link collisions)
        NOTE: archived for main usage, but available for one shot checks if needed
        """
        with Timer(name="OLD Check Link Collision", enabled=False):
            self.logger(f"Target link requested is: {target_link}")
            # Handle invalid link name input
            if target_link == '' or target_link == None or not isinstance(target_link, str):
                self.logger(f"Self Collision Check -> Link name [{target_link}] is invalid.", 'warn')
                return None, False
            
            # Handle check list empty scenario
            if check_list == []:
                # print(f"Check list is empty so terminate.")
                return None, False

            # DEBUGGING
            # rospy.loginfo(f"{target_link} has the following collision objects: {check_list}")
            
            # NOTE iterates over all configured links and compares against provided list of check shapes 
            # NOTE: the less objects in a check list, the better
            #       this is to handle cases (like with the panda) that has multiple shapes per link defining its collision geometry
            #       any custom descriptions should aim to limit the geometry per link as robot geometry is controlled by the vendor
            # NOTE: ignore list is initiased at start up and is meant to handle cases where a mounted table (in collision with the base) is ignored
            #       i.e., does not throw a collision for base_link in collision with table (as it is to be ignored) but will trigger for end-effector link
            for link in reversed(sliced_links):
                # print(f"Link being checked: {link.name}")
                # Check against ignore list and continue if inside
                # NOTE: this assumes that the provided target link (dictating the ignore list) is unique
                #       in some cases the robot's links (if multiple are being checked) may have the same named links
                #       TODO: uncertain if this is scalable (currently working on two pandas with the same link names), but check this
                # NOTE: ignore any links that are expected to be overlapped with current link (inside current robot object)
                if link.name in ignore_list: 
                    # print(f"{link.name} is in list: {ignore_list}, so skipping")
                    continue

                # Ignore check if the target link is the same
                if link.name == target_link:
                    # rospy.logwarn(f"Self Collision Check -> Skipping the current target: {link.name}")
                    continue

                # NOTE: as per note above, ideally this loop should be a oneshot (in most instances)
                # TODO: does it make sense to only check the largest shape in this list? 
                for obj in check_list:
                    # rospy.logwarn(f"LOCAL CHECK for [{self.name}] -> Checking: {link.name}")
                    if link.iscollided(obj, skip=True):
                        self.logger(f"Self Collision Check -> Link that is collided: {link.name}", 'error')
                        return link, True
                
            return None, False

    def neo(self, Tep, velocities):
        """
        Runs a version of Jesse H.'s NEO controller
        <IN DEVELOPMENT>
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
                start=self.link_dict["panda_link1"],
                end=self.link_dict["panda_hand"],
            )

            # print(f"c_Ain: {np.shape(c_Ain)} | Ain: {np.shape(Ain)}")
            # If there are any parts of the robot within the influence distance
            # to the collision in the scene
            if c_Ain is not None and c_bin is not None:
                c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 4))]

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

    def check_collision(self) -> bool:
        """
        High-level check of collision
        NOTE: this is called by loop to verify preempt of robot
        NOTE: This may not be needed as main armer class (high-level) will check collisions per robot
        """
        # Check for collisions
        # NOTE: optimise this as much as possible
        # NOTE: enabled in below Timer line to True for debugging print of frequency of operation
        with Timer(name="Collision Check",enabled=False):
            collision = self.full_collision_check()

        # Handle checking
        if collision and self.preempted == False:
            self.collision_approached = True
            return True
        else:
            self.collision_approached = False
            return False
        
    def set_safe_state(self):
        # Account for pointer location
        if self.q_safe_window_p < len(self.q_safe_window):
            # Add current target bar x value
            self.q_safe_window[self.q_safe_window_p] = self.q
            # increment pointer(s)
            self.q_safe_window_p += 1
        else:
            # shift all values to left by 1 (defaults to 0 at end)
            self.q_safe_window = np.roll(self.q_safe_window, shift=-1, axis=0)
            # add value to end
            self.q_safe_window[-1] = self.q  

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