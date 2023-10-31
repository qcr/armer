"""
ROSRobot module defines the ROSRobot type

.. codeauthor:: Gavin Suddreys
.. codeauthor:: Dasun Gunasinghe
"""
import copy
import os
import timeit
import json

from typing import List, Any
from threading import Lock, Event
from armer.timer import Timer
from armer.trajectory import TrajectoryExecutor
from armer.models import URDFRobot
import rospy
import actionlib
import tf
import roboticstoolbox as rtb
import spatialmath as sm
from spatialmath import SE3, SO3, UnitQuaternion, base
import pointcloud_utils as pclu
import numpy as np
import yaml
# Required for NEO
import qpsolvers as qp
import spatialgeometry as sg

from armer.utils import ikine, mjtg, trapezoidal

from std_msgs.msg import Header, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped

from geometry_msgs.msg import TwistStamped, Twist, Transform
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from std_msgs.msg import Float64MultiArray

from armer_msgs.msg import *
from armer_msgs.srv import *

# pylint: disable=too-many-instance-attributes

import tf2_ros

class ControlMode:
   ERROR=0
   JOINTS=1
   CARTESIAN=2

class ROSRobot(rtb.Robot):
    """
    The ROSRobot class wraps the rtb.ERobot implementing basic ROS functionality
    """

    def __init__(self,
                 robot: rtb.robot.Robot,
                 name: str = None,
                 joint_state_topic: str = None,
                 joint_velocity_topic: str = None,
                 origin=None,
                 config_path=None,
                 readonly=False,
                 frequency=None,
                 modified_qr=None,
                 singularity_thresh=0.02,
                 max_joint_velocity_gain=20.0,
                 max_cartesian_speed=2.0,
                 trajectory_end_cutoff=0.000001,
                 qlim_min=None,
                 qlim_max=None,
                 * args,
                 **kwargs):  # pylint: disable=unused-argument
        
        super().__init__(robot)
        self.__dict__.update(robot.__dict__)
        
        self.name = name if name else self.name
        self.readonly = readonly

        self.max_joint_velocity_gain = max_joint_velocity_gain
        self.max_cartesian_speed = max_cartesian_speed
        self.trajectory_end_cutoff = trajectory_end_cutoff

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.backend_reset = False

        # Update with ustom qlim (joint limits) if specified in robot config
        if qlim_min and qlim_max:
            self.qlim = np.array([qlim_min, qlim_max])
            rospy.loginfo(f"Updating Custom qlim: {self.qlim}")

        # Singularity index threshold (0 is a sigularity)
        # NOTE: this is a tested value and may require configuration (i.e., speed of robot)
        rospy.loginfo(f"[INIT] Singularity Scalar Threshold set to: {singularity_thresh}")
        self.singularity_thresh = singularity_thresh 
        self.manip_scalar = None
        self.singularity_approached = False
        
        if not hasattr(self, 'gripper'):
          self.gripper = self.grippers[0].name if len(self.grippers) > 0 else 'tool0'
          
        # Global list of links
        # NOTE: we currently only consider all links up to our configured self.gripper
        self.sorted_links=[]
        # Check if the existing gripper name exists (Error handling) otherwise default to top of dict stack
        if self.gripper not in self.link_dict.keys():
            default_top_link_name = sorted(self.link_dict.keys())[-1]
            rospy.logwarn(f"Configured gripper name {self.gripper} not in link tree -> defaulting to top of stack: {default_top_link_name}")
            self.gripper = default_top_link_name

        # Sort links by parents starting from gripper
        link=self.link_dict[self.gripper]   
        # print(f"link dict: {sorted(self.link_dict.keys())[-1]}")
        
        # COLLISION HANDLING TESTING
        # NOTE: needs optimisation in future
        # Loops through links (as read in through URDF parser)
        # Extracts each link's list of collision shapes (of type sg.Shape). TODO: add a validity check on type here
        # Updates a class dictionary (key as link name) -> ideally first entry is our gripper (this can change)
        # Create a new dictionary of link lookup to ignore overlapping joint/collision objects
        # NOTE: the theory here is to keep a record of all expected collisions (overlaps) per link so that
        #       the main self collision check can ignore these cases. 
        self.overlapped_link_dict = dict()
        self.collision_dict = dict()
        self.collision_approached = False
        # Iterate through robot links and sort - add to tracked collision list
        while link is not None:
            # Debugging
            # print(f"link name in sort: {link.name}")
            # Add current link to overall dictionary
            self.collision_dict[link.name] = link.collision.data if link.collision.data else []
            self.sorted_links.append(link)
            link=link.parent
        self.sorted_links.reverse()
 
        # Check for external links (from robot tree)
        # This is required to add any other links (not specifically part of the robot tree) 
        # to our collision dictionary for checking
        for link in self.links:
            # print(f"links: {link.name}")
            if link.name in self.collision_dict.keys():
                continue

            # print(f"adding link: {link.name} which is type: {type(link)} with collision data: {link.collision.data}")
            self.collision_dict[link.name] = link.collision.data if link.collision.data else []

        # Debugging
        # print(f"Collision dict for links: {self.collision_dict}\n")
        # print(f"Dictionary of expected link collisions: {self.overlapped_link_dict}\n")    

        self.joint_indexes = []
        self.joint_names = list(map(lambda link: link._joint_name, filter(lambda link: link.isjoint, self.sorted_links)))
        
        if origin:
            self.base = SE3(origin[:3]) @ SE3.RPY(origin[3:])

        self.frequency = frequency if frequency else rospy.get_param((
          joint_state_topic if joint_state_topic else '/joint_states') + '/frequency', 500
        )
        
        self.q = self.qr if hasattr(self, 'qr') else self.q # pylint: disable=no-member
        if modified_qr:
            self.qr = modified_qr
            self.q = modified_qr

        # Joint state message
        self.joint_states = None

        # Guards used to prevent multiple motion requests conflicting
        self._controller_mode = ControlMode.JOINTS

        self.moving: bool = False
        self.last_moving: bool = False
        self.preempted: bool = False

        # Thread variables
        self.lock: Lock = Lock()
        self.event: Event = Event()

        # Arm state property
        self.state: ManipulatorState = ManipulatorState()

        # Expected cartesian velocity
        self.e_v_frame: str = None 

        # cartesian motion
        self.e_v: np.array = np.zeros(shape=(6,)) 
        # expected joint velocity
        self.j_v: np.array = np.zeros(
            shape=(len(self.q),)
        ) 

        self.e_p = self.fkine(self.q, start=self.base_link, end=self.gripper)

        self.last_update: float = 0
        self.last_tick: float = 0

        # Trajectory Generation (designed to expect a Trajectory class obj)
        self.executor = None
        self.traj_generator = mjtg

        self.joint_subscriber = rospy.Subscriber(
            joint_state_topic if joint_state_topic else '/joint_states',
            JointState,
            self._state_cb
        )

        self.joint_velocity_topic = joint_velocity_topic \
                if joint_velocity_topic \
                else '/joint_group_velocity_controller/command'
        
        if not self.readonly:
            # Create Transform Listener
            self.tf_listener = tf.TransformListener()

            # --- Setup Configuration for ARMer --- #
            # NOTE: this path (if not set in the cfg/ARM_real.yaml) does not find the user when run
            #       with systemd. This param must be loaded in via the specific robot_real.yaml file.
            self.config_path = config_path if config_path else os.path.join(
                os.getenv('HOME', '/home'),
                '.ros/configs/system_named_poses.yaml'
            )
            self.custom_configs: List[str] = []
            self.__load_config()

            # --- ROS Publisher Setup --- #
            self.joint_publisher: rospy.Publisher = rospy.Publisher(
                self.joint_velocity_topic,
                Float64MultiArray,
                queue_size=1
            )
            self.state_publisher: rospy.Publisher = rospy.Publisher(
                '{}/state'.format(self.name.lower()), 
                ManipulatorState, 
                queue_size=1
            )
            self.cartesian_servo_publisher: rospy.Publisher = rospy.Publisher(
                '{}/cartesian/servo/arrived'.format(self.name.lower()), 
                Bool, 
                queue_size=1
            )

            # --- ROS Subscriber Setup --- #
            self.cartesian_velocity_subscriber: rospy.Subscriber = rospy.Subscriber(
                '{}/cartesian/velocity'.format(self.name.lower()), 
                TwistStamped, 
                self.velocity_cb
            )
            self.joint_velocity_subscriber: rospy.Subscriber = rospy.Subscriber(
                '{}/joint/velocity'.format(self.name.lower()), 
                JointVelocity, 
                self.joint_velocity_cb
            )
            self.cartesian_servo_subscriber: rospy.Subscriber = rospy.Subscriber(
                '{}/cartesian/servo'.format(self.name.lower()), 
                ServoStamped, 
                self.servo_cb
            )
            self.set_pid_subscriber: rospy.Subscriber = rospy.Subscriber(
                '{}/set_pid'.format(self.name.lower()),
                Float64MultiArray,
                self.set_pid
            )

            # --- ROS Action Server Setup --- #
            self.velocity_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
                '{}/cartesian/guarded_velocity'.format(self.name.lower()),
                GuardedVelocityAction,
                execute_cb=self.guarded_velocity_cb,
                auto_start=False
            )
            self.velocity_server.register_preempt_callback(self.preempt)
            self.velocity_server.start()

            self.pose_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
                '{}/cartesian/pose'.format(self.name.lower()),
                MoveToPoseAction,
                execute_cb=self.pose_cb,
                auto_start=False
            )
            self.pose_server.register_preempt_callback(self.preempt)
            self.pose_server.start()

            self.step_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
                '{}/cartesian/step'.format(self.name.lower()),
                MoveToPoseAction,
                execute_cb=self.step_cb,
                auto_start=False
            )
            self.step_server.register_preempt_callback(self.preempt)
            self.step_server.start()

            self.pose_tracking_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
                '{}/cartesian/track_pose'.format(self.name.lower()),
                TrackPoseAction,
                execute_cb=self.pose_tracker_cb,
                auto_start=False,
            )
            self.pose_tracking_server.register_preempt_callback(self.preempt_tracking)
            self.pose_tracking_server.start()

            # TODO: Remove this action server - just for debugging
            self.tf_to_pose_transporter_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
                '{}/pose_from_tf'.format(self.name.lower()),
                TfToPoseAction,
                execute_cb=self.tf_to_pose_transporter_cb,
                auto_start=False,
            )
            self.tf_to_pose_transporter_server.register_preempt_callback(self.preempt_other)
            self.tf_to_pose_transporter_server.start()

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

            self.named_pose_in_frame_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
                '{}/joint/named_in_frame'.format(self.name.lower()),
                MoveToNamedPoseAction,
                execute_cb=self.named_pose_in_frame_cb,
                auto_start=False
            )
            self.named_pose_in_frame_server.register_preempt_callback(self.preempt)
            self.named_pose_in_frame_server.start()

            self.named_pose_distance_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
                '{}/measurement/named_to_gripper'.format(self.name.lower()),
                MoveToNamedPoseAction,
                execute_cb=self.named_pose_distance_cb,
                auto_start=False
            )
            self.named_pose_distance_server.register_preempt_callback(self.preempt)
            self.named_pose_distance_server.start()

            self.home_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
                '{}/home'.format(self.name.lower()),
                HomeAction,
                execute_cb=self.home_cb,
                auto_start=False
            )
            self.home_server.register_preempt_callback(self.preempt)
            self.home_server.start()

            # --- ROS Services Setup --- #            
            rospy.Service(
                '{}/recover'.format(self.name.lower()),
                Empty, 
                self.recover_cb
            )
            
            rospy.Service(
                '{}/stop'.format(self.name.lower()),
                Empty, 
                self.preempt
            )
            
            rospy.Service(
                '{}/set_cartesian_impedance'.format(self.name.lower()),
                SetCartesianImpedance,
                self.set_cartesian_impedance_cb
            )

            rospy.Service(
                '{}/get_ee_link_name'.format(self.name.lower()),
                GetLinkName,
                lambda req: GetLinkNameResponse(name=self.gripper)
            )

            rospy.Service(
                '{}/get_base_link_name'.format(self.name.lower()),
                GetLinkName,
                lambda req: GetLinkNameResponse(name=self.base_link.name)
            )

            rospy.Service(
                '{}/update_description'.format(self.name.lower()),
                UpdateDescription,
                self.update_description_cb
            )

            rospy.Service(
                '{}/calibrate_transform'.format(self.name.lower()),
                CalibrateTransform,
                self.calibrate_transform_cb
            )

            rospy.Service(
                '{}/get_named_poses'.format(self.name.lower()), 
                GetNamedPoses,
                self.get_named_poses_cb
            )
            
            rospy.Service(
                '{}/set_named_pose'.format(self.name.lower()), 
                AddNamedPose,
                self.add_named_pose_cb
            )

            rospy.Service(
                '{}/set_named_pose_in_frame'.format(self.name.lower()), 
                AddNamedPoseInFrame,
                self.add_named_pose_in_frame_cb
            )
            
            rospy.Service(
                '{}/remove_named_pose'.format(self.name.lower()), 
                RemoveNamedPose,
                self.remove_named_pose_cb
            )
            
            rospy.Service(
                '{}/export_named_pose_config'.format(self.name.lower()),
                NamedPoseConfig,
                self.export_named_pose_config_cb
            )

            rospy.Service(
                '{}/add_named_pose_config'.format(self.name.lower()),
                NamedPoseConfig,
                self.add_named_pose_config_cb
            )

            rospy.Service(
                '{}/remove_named_pose_config'.format(self.name.lower()),
                NamedPoseConfig,
                self.remove_named_pose_config_cb
            )
            
            rospy.Service(
                '{}/get_named_pose_configs'.format(self.name.lower()),
                GetNamedPoseConfigs,
                self.get_named_pose_configs_cb
            )

    # --------------------------------------------------------------------- #
    # --------- ROS Topic Callback Methods -------------------------------- #
    # --------------------------------------------------------------------- #
    def _state_cb(self, msg):
        if not self.joint_indexes:
            for joint_name in self.joint_names:
                self.joint_indexes.append(msg.name.index(joint_name))
        
        self.q = np.array(msg.position)[self.joint_indexes] if len(msg.position) == self.n else np.zeros(self.n)
        self.joint_states = msg
        
    def velocity_cb(self, msg: TwistStamped) -> None:
        """
        ROS velocity callback:
        Moves the arm at the specified cartesian velocity
        w.r.t. a target frame

        :param msg: [description]
        :type msg: TwistStamped
        """
        if self._controller_mode == ControlMode.ERROR:
            rospy.logerr(f"CART VEL CB: [{self.name}] in Error Control Mode...")
            return None

        if self.moving:
            print(f"HERE because moving (CART)")
            self.preempt()

        with self.lock:
            self.preempted = False
            self.__vel_move(msg)

    def joint_velocity_cb(self, msg: JointVelocity) -> None:
        """
        ROS joint velocity callback:
        Moves the joints of the arm at the specified velocities

        :param msg: [description]
        :type msg: JointVelocity
        """
        if self._controller_mode == ControlMode.ERROR:
            rospy.logerr(f"JOINT VEL CB: [{self.name}] in Error Control Mode...")
            return None

        # Check for vector length and terminate if invalid
        if len(msg.joints) != len(self.j_v):
            rospy.logerr(f"JOINT VEL CB: [{self.name}] provided input vector length invalid [{len(msg.joints)}]. Expecting len [{len(self.j_v)}]")
            return None
         
        if self.moving:            
            print(f"HERE because moving (JOINT)")
            self.preempt()

        with self.lock:
            self.j_v = np.array(msg.joints)
            self.last_update = rospy.get_time()

    # --------------------------------------------------------------------- #
    # --------- ROS Action Callback Methods ------------------------------- #
    # --------------------------------------------------------------------- #
    def guarded_velocity_cb(self, msg: GuardedVelocityGoal) -> None:
        """
        ROS Guarded velocity callback
        Moves the end-effector in cartesian space with respect to guards (time or force)
        
        :param msg: [description]
        :type msg: GuardedVelocityGoal
        """
        if self.moving:
            self.preempt()
        
        with self.lock:
            self.preempted = False
            
            start_time = rospy.get_time()
            triggered = 0
            
            while not self.preempted:
                triggered = self.test_guards(msg.guards, start_time=start_time)

                if triggered != 0:
                    break

                self.__vel_move(msg.twist_stamped)
                rospy.sleep(0.01)

            if not self.preempted:
                self.velocity_server.set_succeeded(GuardedVelocityResult(triggered=triggered))
            else:
                self.velocity_server.set_aborted(GuardedVelocityResult())

    def servo_cb(self, msg) -> None:
        """
        ROS Servoing Action Callback:
        Servos the end-effector to the cartesian pose given by msg
        
        :param msg: [description]
        :type msg: ServoStamped

        This callback makes use of the roboticstoolbox p_servo function
        to generate velocities at each timestep.
        """
        # Safely stop any current motion of the arm
        if self.moving:
            self.preempt()
        
        with self.lock:
            # Handle variables for servo
            goal_pose = msg.pose
            goal_gain = msg.gain if msg.gain else 0.2
            goal_thresh = msg.threshold if msg.threshold else 0.005
            arrived = False
            self.moving = True
            self.preempted = False

            # Current end-effector pose
            Te = self.ets(start=self.base_link, end=self.gripper).eval(self.q)

            # Handle frame id of servo request
            if msg.header.frame_id == '':
                msg.header.frame_id = self.base_link.name
            
            goal_pose_stamped = self.tf_listener.transformPose(
                self.base_link.name,
                PoseStamped(header=msg.header, pose=goal_pose)
            )
            pose = goal_pose_stamped.pose

            # Convert target to SE3 (from pose)
            target = SE3(pose.position.x, pose.position.y, pose.position.z) * UnitQuaternion([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ]).SE3()

            # Calculate the required end-effector spatial velocity for the robot
            # to approach the goal.
            velocities, arrived = rtb.p_servo(
                Te,
                target,
                min(20, goal_gain),
                threshold=goal_thresh
            )

            ##### TESTING NEO IMPLEMENTATION #####
            # neo_jv = self.neo(Tep=target, velocities=velocities)
            neo_jv = None

            if np.any(neo_jv):
                self.j_v = neo_jv[:len(self.q)]
            else:
                self.j_v = np.linalg.pinv(self.jacobe(self.q)) @ velocities

            # print(f"current jv: {self.j_v} | updated neo jv: {neo_jv}")
            self.last_update = rospy.get_time()

        self.cartesian_servo_publisher.publish(arrived)

    def tf_to_pose_transporter_cb(self, goal: TfToPoseGoal) -> None:
        pub_rate = rospy.Rate(goal.rate)
        pub = rospy.Publisher(goal.pose_topic, PoseStamped, queue_size=1)
        pub_target = rospy.Publisher("/target_target", Pose, queue_size=1)
        pub_ee = rospy.Publisher("/target_ee", Pose, queue_size=1)

        rospy.logerr("Updating TF to Pose publisher...")

        previous_pose = Pose()

        while not self.tf_to_pose_transporter_server.is_preempt_requested():

            if goal.target_tf == None or goal.target_tf == "":
                rospy.logerr(f"Provided target tf is None")
                pub_rate.sleep()
                continue

            ee_pose = Pose()
            target_pose_offset = Pose()
            try:
                self.tf_listener.waitForTransform(self.base_link.name, "ee_control_link", rospy.Time.now(), rospy.Duration(1.0))
                ee_pose = pclu.ROSHelper().tf_to_pose(self.base_link.name, "ee_control_link", self.tfBuffer)
                self.tf_listener.waitForTransform(goal.ee_frame, goal.target_tf, rospy.Time.now(), rospy.Duration(1.0))
                target_pose_offset = pclu.ROSHelper().tf_to_pose(goal.ee_frame, goal.target_tf, self.tfBuffer)
            except:
                rospy.logerr(f"failed to get transforms...")
                if previous_pose != Pose():
                    pub.publish(previous_pose)
                    rospy.logerr(f"...previous_pose available")
                elif ee_pose != Pose():
                    previous_pose = PoseStamped()
                    previous_pose.header.stamp = rospy.Time.now()
                    previous_pose.header.frame_id = self.base_link.name
                    previous_pose.pose = ee_pose
                    pub.publish(previous_pose)
                    rospy.logerr(f"...ee_pose available")
                pub_rate.sleep()
                continue
                    

            if target_pose_offset == Pose():
                rospy.logerr(f"target vector is empty, cannot be calculated. exiting...")
                if previous_pose != Pose():
                    pub.publish(previous_pose)
                elif ee_pose != Pose():
                    previous_pose = PoseStamped()
                    previous_pose.header.stamp = rospy.Time.now()
                    previous_pose.header.frame_id = self.base_link.name
                    previous_pose.pose = ee_pose
                    pub.publish(previous_pose)
                pub_rate.sleep()
                continue

            target_rotation = sm.UnitQuaternion(
                target_pose_offset.orientation.w, [
                target_pose_offset.orientation.x,
                target_pose_offset.orientation.y,
                target_pose_offset.orientation.z
            ])

            ee_rotation = sm.UnitQuaternion(
                ee_pose.orientation.w, [
                ee_pose.orientation.x,
                ee_pose.orientation.y,
                ee_pose.orientation.z
            ])

            # SO3 representations
            target_rot_so = sm.SO3.RPY(target_rotation.rpy(order='xyz')) # Working...
            ee_rot_so = sm.SO3.RPY(ee_rotation.rpy(order='zyx'))

            goal_rotation = sm.UnitQuaternion(ee_rot_so * target_rot_so.inv()) # works perfect with neg R & P

            # Hack test to negate R & P should apply rotation
            goal_rpy = goal_rotation.rpy()
            goal_rotation = sm.UnitQuaternion(sm.SO3.RPY(-goal_rpy[0], -goal_rpy[1], goal_rpy[2]))

            goal_pose = copy.deepcopy(ee_pose)
            goal_pose.orientation.w = goal_rotation.s
            goal_pose.orientation.x = goal_rotation.v[0]
            goal_pose.orientation.y = goal_rotation.v[1]
            goal_pose.orientation.z = goal_rotation.v[2]

            # This is a hack...fix it...
            # NOTE: this assumes target is in the camera frame...
            # Use SE3 and apply the above rotations
            goal_pose.position.x += target_pose_offset.position.y
            goal_pose.position.y += target_pose_offset.position.z
            goal_pose.position.z += target_pose_offset.position.x

            rospy.logwarn(f"TF to Pose")
            rospy.logwarn(f"-- Current EE Pose AS RPY {ee_rot_so.rpy()}")
            rospy.logwarn(f"-- Target AS RPY {target_rot_so.rpy()}")
            rospy.logwarn(f"-- RESULT AS RPY {goal_rotation.rpy()}")
            rospy.logwarn(f"-- Goal EE Pose {goal_pose}")

            pub_target.publish(target_pose_offset)
            pub_ee.publish(ee_pose)

            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.stamp = rospy.Time.now()
            goal_pose_msg.header.frame_id = self.base_link.name
            goal_pose_msg.pose = goal_pose
            pub.publish(goal_pose_msg)

            previous_pose = copy.deepcopy(goal_pose_msg)

            # break
            pub_rate.sleep()

        self.tf_to_pose_transporter_server.set_succeeded(
            TfToPoseResult(success=True))

    def pose_tracker_cb(self, goal: TrackPoseGoal) -> None:
        """
        ROS Action Server callback:
        Moves the end-effector to the
        cartesian pose defined by the supplied PoseStamped topic
        """
        msg_pose_publish_rate = 10
        pub_rate = rospy.Rate(msg_pose_publish_rate)

        # TODO: Refactor - this should be used both for distance to goal and for new pose validation
        msg_pose_threshold = 0.005

        msg_pose_topic = goal.tracked_pose_topic

        # TODO: Remove debugging
        feedback = TrackPoseFeedback()
        feedback.status = 0
        self.pose_tracking_server.publish_feedback(feedback)

        if self.moving:
            self.preempt()
            self.moving = False

        with self.lock:
            self.preempted = False

            tracked_pose = None

            # while not self.preempted: 
            while not self.pose_tracking_server.is_preempt_requested():

                if self.preempted:
                    # Reset the board
                    self.preempted = False
                    # - do some other things...like control_type

                pose_msg = None
                try:
                    pose_msg = rospy.wait_for_message(topic=msg_pose_topic, topic_type=PoseStamped, timeout=0.1)
                    # Apply offset provided to pose_msg
                    pose_msg.pose.position.x += goal.pose_offset_x
                    pose_msg.pose.position.y += goal.pose_offset_y
                    pose_msg.pose.position.z += goal.pose_offset_z

                    tracked_pose = copy.deepcopy(pose_msg)
                except:
                    pose_msg = tracked_pose

                if pose_msg:
                    goal_pose = pose_msg
                elif tracked_pose is None:
                    # Not currently tracking
                    pub_rate.sleep()
                    continue

                # TODO: Remove debugging
                feedback = TrackPoseFeedback()
                feedback.status = 2
                self.pose_tracking_server.publish_feedback(feedback)

                # TODO: Check if we bother processing the goal_pose given the tracked_pose distance
                tracked_pose = goal_pose

                # IS THE GOAL POSE WITHIN THE BOUNDRY?...
                if self.pose_within_workspace(goal_pose.pose) == False:
                    pub_rate.sleep()
                    continue

                if goal.linear_motion:
                    # Method 2: Use Servo to Pose
                    # Handle variables for servo
                    goal_gain = goal.vel_scale * self.max_joint_velocity_gain if goal.vel_scale else 0.2
                    goal_thresh = msg_pose_threshold if msg_pose_threshold else 0.005
                    arrived = False
                    self.moving = True

                    # Current end-effector pose
                    Te = self.ets(start=self.base_link, end=self.gripper).eval(self.q)

                    pose = goal_pose.pose

                    # Overwrite provided pose orientation if required
                    if goal.gripper_orientation_lock:
                        pose.orientation = self.state.ee_pose.pose.orientation

                    # Convert target to SE3 (from pose)
                    target = SE3(pose.position.x, pose.position.y, pose.position.z) * UnitQuaternion(
                        pose.orientation.w,
                        [pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z
                    ]).SE3()

                    # Calculate the required end-effector spatial velocity for the robot
                    # to approach the goal.
                    velocities, arrived = rtb.p_servo(
                        Te,
                        target,
                        min(20, goal_gain),
                        threshold=goal_thresh
                    )

                    ## TODO: Remove this or NEO Testing...
                    self.j_v = np.linalg.pinv(self.jacobe(self.q)) @ velocities
                    self.last_update = rospy.get_time()

                    # ##### TESTING NEO IMPLEMENTATION #####
                    # # neo_jv = self.neo(Tep=target, velocities=velocities)
                    # neo_jv = None

                    # if np.any(neo_jv):
                    #     self.j_v = neo_jv[:len(self.q)]
                    # else:
                    #     self.j_v = np.linalg.pinv(self.jacobe(self.q)) @ velocities

                    # # print(f"current jv: {self.j_v} | updated neo jv: {neo_jv}")
                    # self.last_update = rospy.get_time()

                    # # if arrived == True:
                    # #     break

                    pub_rate.sleep()
                    # # ---- END Method 2 ---------------------------------
                else:
                    # Method 1: Use TrajectoryExecutor
                    goal_speed = goal.vel_scale * self.max_cartesian_speed if goal.vel_scale else 0.2
                    if goal_pose.header.frame_id == '':
                        goal_pose.header.frame_id = self.base_link.name

                    goal_pose = self.tf_listener.transformPose(
                        self.base_link.name,
                        goal_pose,
                    )

                    pose = goal_pose.pose

                    # Test Distance to goal and for active motion
                    # TODO: Add pose comparison
                    if self.executor is not None:
                        if not self.executor.is_finished(cutoff=msg_pose_threshold):
                            pub_rate.sleep()
                            continue
                        # if not self.executor.is_succeeded():
                        #     self.executor = None
                        #     pub_rate.sleep()
                        #     continue

                    solution = None
                    try:
                        solution = ikine(self, pose, q0=self.q, end=self.gripper)
                    except:
                        rospy.logwarn("Failed to get IK Solution...")

                    if solution is None:
                        pub_rate.sleep()
                        continue

                #     # Check for singularity on end solution:
                #     # TODO: prevent motion on this bool? Needs to be thought about
                #     if self.check_singularity(solution.q):
                #         rospy.logwarn(f"IK solution within singularity threshold [{self.singularity_thresh}] -> ill-advised motion")

                    try:
                        self.executor = TrajectoryExecutor(
                        self,
                        self.traj_generator(self, solution.q, goal_speed),
                        cutoff=self.trajectory_end_cutoff
                        )
                    except:
                        rospy.logwarn("TrackPose - Unable to construct TrajectoryExecutor")
                        pub_rate.sleep()
                        continue

                    pub_rate.sleep()
                    # ---- END Method 1 ---------------------------------

        # TODO: Remove debugging
        feedback = TrackPoseFeedback()
        feedback.status = 3
        self.pose_tracking_server.publish_feedback(feedback)

        if not self.preempted:
            # TODO: Remove debugging
            feedback = TrackPoseFeedback()
            feedback.status = 33
            self.pose_tracking_server.publish_feedback(feedback)

            self.pose_tracking_server.set_succeeded(TrackPoseResult(success=True))
        else:
            # TODO: Remove debugging
            feedback = TrackPoseFeedback()
            feedback.status = 34
            self.pose_tracking_server.publish_feedback(feedback)

            self.pose_tracking_server.set_aborted(TrackPoseResult(success=False))

        self.executor = None
        self.moving = False

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
                goal_pose,
            )
            
            pose = goal_pose.pose

            # IS THE GOAL POSE WITHIN THE BOUNDRY?...
            if self.pose_within_workspace(pose) == False:
                rospy.logwarn("-- Pose goal outside defined workspace; refusing to move...")
                self.pose_server.set_succeeded(
                  MoveToPoseResult(success=False), 'Named pose outside defined workspace'
                )
                self.executor = None
                self.moving = False
                return
            
            solution = ikine(self, pose, q0=self.q, end=self.gripper)

            # Check for singularity on end solution:
            # TODO: prevent motion on this bool? Needs to be thought about
            if self.check_singularity(solution.q):
                rospy.logwarn(f"IK solution within singularity threshold [{self.singularity_thresh}] -> ill-advised motion")
            
            self.executor = TrajectoryExecutor(
                self,
                self.traj_generator(self, qf=solution.q, max_speed=goal.speed if goal.speed else 0.2),
                cutoff=self.trajectory_end_cutoff
            )

            while not self.executor.is_finished():
              rospy.sleep(0.01)

            if self.executor.is_succeeded():
                self.pose_server.set_succeeded(MoveToPoseResult(success=True))
            else:
                self.pose_server.set_aborted(MoveToPoseResult(success=False))

            self.executor = None
            self.moving = False

    def step_cb(self, goal: MoveToPoseGoal) -> None:
        """
        """
        if self.moving:
            self.preempt()
            self.moving = False

        with self.lock:
            arrived = False
            self.moving = True
            
            goal_pose = goal.pose_stamped

            if goal_pose.header.frame_id == '':
                goal_pose.header.frame_id = self.base_link.name

            # Get the EE pose in the armer defined base_link
            ee_pose = self.ets(start=self.base_link, end=self.gripper).eval(self.q.tolist())
            ee_pose_stamped = PoseStamped()
            ee_pose_stamped.header.frame_id = self.base_link.name

            translation = ee_pose[:3, 3]
            ee_pose_stamped.pose.position.x = translation[0]
            ee_pose_stamped.pose.position.y = translation[1]
            ee_pose_stamped.pose.position.z = translation[2]

            rotation = ee_pose[:3, :3]
            ee_rot = sm.UnitQuaternion(rotation)
            ee_pose_stamped.pose.orientation.w = ee_rot.A[0]
            ee_pose_stamped.pose.orientation.x = ee_rot.A[1]
            ee_pose_stamped.pose.orientation.y = ee_rot.A[2]
            ee_pose_stamped.pose.orientation.z = ee_rot.A[3]

            # Transform EE to goal frame
            ee_in_goal = self.tf_listener.transformPose(
                goal.pose_stamped.header.frame_id,
                ee_pose_stamped,
            )

            # Apply goal step
            step_pose_stamped = copy.deepcopy(goal_pose)
            step_pose_stamped.pose.position.x += ee_in_goal.pose.position.x
            step_pose_stamped.pose.position.y += ee_in_goal.pose.position.y
            step_pose_stamped.pose.position.z += ee_in_goal.pose.position.z

            # NOTE: Ignore orientation until an action message is made with degrees for user convenience
            step_pose_stamped.pose.orientation.w = ee_in_goal.pose.orientation.w
            step_pose_stamped.pose.orientation.x = ee_in_goal.pose.orientation.x
            step_pose_stamped.pose.orientation.y = ee_in_goal.pose.orientation.y
            step_pose_stamped.pose.orientation.z = ee_in_goal.pose.orientation.z

            # Transform step_pose to armer base_link
            step_pose_stamped = self.tf_listener.transformPose(
                self.base_link.name,
                step_pose_stamped,
            )
            step_pose = step_pose_stamped.pose

            # IS THE GOAL POSE WITHIN THE BOUNDRY?...
            if self.pose_within_workspace(step_pose) == False:
                rospy.logwarn("-- Pose goal outside defined workspace; refusing to move...")
                self.step_server.set_succeeded(
                  MoveToPoseResult(success=False), 'Named pose outside defined workspace'
                )
                self.executor = None
                self.moving = False
                return
            
            # TODO: Refactor - this provided as a parameter
            msg_pose_threshold = 0.005
            
            if goal.linear_motion:
                rospy.logwarn('Moving in linear motion mode...')
                # Method 2: Use Servo to Pose
                # Handle variables for servo
                goal_gain = goal.speed * self.max_joint_velocity_gain if goal.speed else 0.02 * self.max_joint_velocity_gain
                goal_thresh = msg_pose_threshold if msg_pose_threshold else 0.005

                # Target is just the pose delta provided (orientation currently ignored - remains at current orientation)
                target = SE3(step_pose.position.x, step_pose.position.y, step_pose.position.z) * UnitQuaternion(
                    ee_rot.A[0],
                    [ee_rot.A[1],
                    ee_rot.A[2],
                    ee_rot.A[3]
                ]).SE3()

                rospy.logdebug(f'Goal Pose: {goal_pose.pose}')
                rospy.logdebug(f'Step Pose: {step_pose}')
                rospy.logdebug(f'Target Pose: {target}')

                # Block while move is completed
                while arrived == False and not self.step_server.is_preempt_requested():
                    # Current end-effector pose
                    Te = self.ets(start=self.base_link, end=self.gripper).eval(self.q)

                    # Calculate the required end-effector spatial velocity for the robot
                    # to approach the goal.
                    velocities, arrived = rtb.p_servo(
                        Te,
                        target,
                        min(20, goal_gain),
                        threshold=goal_thresh
                    )

                    # TODO: Investigate / Validate returned arrived boolean from RTB
                    # - default currently is RPY method
                    # - arrived is not arrived (sum of errors < threshold)
                    # - may also mix spatial and angle errors

                    ## TODO: Remove this or NEO Testing...
                    self.j_v = np.linalg.pinv(self.jacobe(self.q)) @ velocities
                    self.last_update = rospy.get_time()
                    rospy.sleep(0.1)

            else:
                solution = ikine(self, step_pose, q0=self.q, end=self.gripper)

                # Check for singularity on end solution:
                # TODO: prevent motion on this bool? Needs to be thought about
                if self.check_singularity(solution.q):
                    rospy.logwarn(f"IK solution within singularity threshold [{self.singularity_thresh}] -> ill-advised motion")
                
                self.executor = TrajectoryExecutor(
                    self,
                    self.traj_generator(self, solution.q, goal.speed if goal.speed else 0.2),
                    cutoff=self.trajectory_end_cutoff
                    )

                # Block while move is completed
                while not self.executor.is_finished() and not self.step_server.is_preempt_requested():
                    rospy.sleep(0.01)

            if (self.executor is not None and self.executor.is_succeeded()) or arrived == True:
                self.step_server.set_succeeded(MoveToPoseResult(success=True))
                rospy.logwarn('...Motion Complete!')
            else:
                self.step_server.set_aborted(MoveToPoseResult(success=False))
                rospy.logwarn('...Failed Motion!')

            # Clean up
            self.executor = None
            self.moving = False

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

            # Check for singularity on end solution:
            # TODO: prevent motion on this bool? Needs to be thought about
            if self.check_singularity(goal.joints):
                rospy.logwarn(f"IK solution within singularity threshold [{self.singularity_thresh}] -> ill-advised motion")

            self.executor = TrajectoryExecutor(
              self,
              self.traj_generator(self, np.array(goal.joints), goal.speed if goal.speed else 0.2),
              cutoff=self.trajectory_end_cutoff
            )

            while not self.executor.is_finished():
              rospy.sleep(0.01)

            if self.executor.is_succeeded():
                self.joint_pose_server.set_succeeded(
                    MoveToJointPoseResult(success=True)
                )
            else:
                self.joint_pose_server.set_aborted(
                    MoveToJointPoseResult(success=False)
                )

            self.executor = None
            self.moving = False

    def named_pose_in_frame_cb(self, goal: MoveToNamedPoseGoal) -> None:
        """
        
        """
        if self.moving:
            self.preempt()

        with self.lock:
            named_poses = {}

            # TODO: clean this up...
            # Defaults to /home/qcr/.ros/configs/system_named_poses.yaml
            # config_file = self.config_path if not self.custom_configs else self.custom_configs[-1]
            config_file = '/home/qcr/armer_ws/src/armer_descriptions/data/custom/cgras_descriptions/config/named_poses.yaml'
            config_file = config_file.replace('.yaml', '_in_frame.yaml')

            try:
                config = yaml.load(open(config_file), Loader=yaml.SafeLoader)
                if config and 'named_poses' in config:
                    named_poses = config['named_poses']
            except IOError:
                rospy.logwarn(
                    'Unable to locate configuration file: {}'.format(config_file))
                self.named_pose_in_frame_server.set_aborted(
                    MoveToNamedPoseResult(success=False),
                    'Unable to locate configuration file: {}'.format(config_file)
                )
                return           

            if goal.pose_name not in named_poses:
                self.named_pose_in_frame_server.set_aborted(
                    MoveToNamedPoseResult(success=False),
                    'Unknown named pose'
                )
                rospy.logwarn(f"-- Named pose goal ({goal.pose_name}) is unknown; refusing to move...")
                return

            # TODO: YAML yuck...
            the_pose = named_poses[goal.pose_name]
            frame_id = the_pose['frame_id']
            translation = the_pose['position']
            orientation = the_pose['orientation']

            ## named PoseStamped position
            header = Header()
            header.frame_id = frame_id

            pose_stamped = PoseStamped()
            pose_stamped.header = header
  
            pose_stamped.pose.position.x = translation[0]
            pose_stamped.pose.position.y = translation[1]
            pose_stamped.pose.position.z = translation[2]

            pose_stamped.pose.orientation.w = orientation[0]
            pose_stamped.pose.orientation.x = orientation[1]
            pose_stamped.pose.orientation.y = orientation[2]
            pose_stamped.pose.orientation.z = orientation[3]

            # Transform into the current base_link ready for inv kin
            # TODO: base_link here should come from self.base_link (assuming this is base_link)
            goal_pose = self.tf_listener.transformPose(
                        f'/{self.base_link.name}',
                        pose_stamped,
                    )

            pose = goal_pose.pose

            rospy.logdebug(f"Named Pose In Frame ---\nFROM: {pose_stamped}")
            rospy.logdebug(f"Named Pose In Frame ---\nTO POSE: {goal_pose}")

            solution = None
            try:
                solution = ikine(self, target=pose, q0=self.q, end=self.gripper)
            except:
                rospy.logwarn("Failed to get IK Solution...")
                self.named_pose_in_frame_server.set_succeeded(
                  MoveToNamedPoseResult(success=False), f'Failed to solve for Named pose ({goal.pose_name}) in frame: {frame_id}'
                )
                self.executor = None
                self.moving = False
                return
            
            rospy.logdebug(f"Named Pose In Frame ---\nTO JOINTS: {solution.q.tolist()}")

            # TODO: BOB FIX THIS...
            if self.pose_within_workspace(pose) == False:
                rospy.logwarn(f"-- Named pose ({goal.pose_name}) goal outside defined workspace; refusing to move...")
                self.named_pose_in_frame_server.set_succeeded(
                  MoveToNamedPoseResult(success=False), f'Named pose ({goal.pose_name}) outside defined workspace using frame: {frame_id}'
                )
                self.executor = None
                self.moving = False
                return

            # NORMAL....            
            self.executor = TrajectoryExecutor(
                self,
                self.traj_generator(self, solution.q, goal.speed if goal.speed else 0.2),
                cutoff=self.trajectory_end_cutoff
            )

            while not self.executor.is_finished():
                rospy.sleep(0.01)

            if self.executor.is_succeeded():
                self.named_pose_in_frame_server.set_succeeded(
                        MoveToNamedPoseResult(success=True)
                )
            else:
                self.named_pose_in_frame_server.set_aborted(
                  MoveToNamedPoseResult(success=False)
                )

            self.executor = None
            self.moving = False
    
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
                    MoveToNamedPoseResult(success=False),
                    'Unknown named pose'
                )
                rospy.logwarn(f"-- Named pose goal ({goal.pose_name}) is unknown; refusing to move...")
                return

            qd = np.array(self.named_poses[goal.pose_name])

            # IS THE GOAL POSE WITHIN THE BOUNDRY?...
            goal_pose_se3 = SE3(self.ets(start=self.base_link, end=self.gripper).eval(qd))
            goal_pose = Pose()
            goal_pose.position.x = goal_pose_se3.t[0]
            goal_pose.position.y = goal_pose_se3.t[1]
            goal_pose.position.z = goal_pose_se3.t[2]

            if self.pose_within_workspace(goal_pose) == False:
                rospy.logwarn("-- Named pose goal outside defined workspace; refusing to move...")
                self.named_pose_server.set_succeeded(
                  MoveToNamedPoseResult(success=False), 'Named pose outside defined workspace'
                )
                self.executor = None
                self.moving = False
                return
            
            self.executor = TrajectoryExecutor(
                self,
                self.traj_generator(self, qd, goal.speed if goal.speed else 0.2),
                cutoff=self.trajectory_end_cutoff
            )

            while not self.executor.is_finished():
                rospy.sleep(0.01)

            if self.executor.is_succeeded():
                self.named_pose_server.set_succeeded(
                        MoveToNamedPoseResult(success=True)
                )
            else:
                self.named_pose_server.set_aborted(
                  MoveToNamedPoseResult(success=False)
                )

            self.executor = None
            self.moving = False

    def named_pose_distance_cb(self, goal: MoveToNamedPoseGoal) -> None:
        """        
        """
        # TODO: should use a custom message (speed should be max cart dist)
        proximity_limit = np.array([goal.speed]*3)

        if not goal.pose_name in self.named_poses:
            self.named_pose_distance_server.set_aborted(
                MoveToNamedPoseResult(success=False),
                'Unknown named pose'
            )

        qd = np.array(self.named_poses[goal.pose_name])
        start_SE3 = SE3(self.ets(start=self.base_link, end=self.gripper).eval(self.q))
        end_SE3 = SE3(self.ets(start=self.base_link, end=self.gripper).eval(qd))

        difference_XYZ = start_SE3.t - end_SE3.t
        rospy.logdebug(f"QD is {qd}")
        rospy.logdebug(f"Q is {self.q}")
        rospy.logdebug(f"Links are {self.base_link} (base) and {self.gripper} (gripper)")
        rospy.logdebug(f"Start is {start_SE3.t}")
        rospy.logdebug(f"End is {end_SE3.t}")
        rospy.logdebug(f"Distance to named pose {goal.pose_name} is {difference_XYZ}")
        exceeded_limit = difference_XYZ > proximity_limit
        if any(exceeded_limit):
            self.named_pose_distance_server.set_succeeded(
                    MoveToNamedPoseResult(success=True))
        else:
            self.named_pose_distance_server.set_succeeded(
                    MoveToNamedPoseResult(success=False))

    def home_cb(self, goal: HomeGoal) -> HomeResult:
        """[summary]

        :param req: Empty request
        :type req: EmptyRequest
        :return: Empty response
        :rtype: EmptyResponse
        """
        if self.moving:
            self.preempt()
            
        with self.lock:
            qd = np.array(self.qr) if hasattr(self, 'qr') else self.q

            rospy.logerr("HOME - pre TrajectoryExecuter loop...")
            
            self.executor = TrajectoryExecutor(
                self,
                self.traj_generator(self, qd, goal.speed if goal.speed else 0.2),
                cutoff=0.01,
            )
            rospy.logerr("HOME - post TrajectoryExecuter loop...")

            rospy.logerr("HOME - pre is_finished loop...")
            while not self.executor.is_finished():
              rospy.sleep(0.01)
            rospy.logerr("HOME - post is_finished loop...")

            if self.executor.is_succeeded():
                self.home_server.set_succeeded(
                    HomeResult(success=True)
                )
            else:
                self.home_server.set_aborted(
                  HomeResult(success=False)
                )

            # Recover from Error if set
            if self._controller_mode == ControlMode.ERROR:
                rospy.loginfo(f"Resetting from ERROR state to JOINTS [Default]")
                self._controller_mode = ControlMode.JOINTS
                self.preempted = False

            self.executor = None
            self.moving = False

    # --------------------------------------------------------------------- #
    # --------- ROS Service Callback Methods ------------------------------- #
    # --------------------------------------------------------------------- #
    def recover_cb(self, req: EmptyRequest) -> EmptyResponse: # pylint: disable=no-self-use
        """[summary]
        ROS Service callback:
        Invoke any available error recovery functions on the robot when an error occurs

        :param req: an empty request
        :type req: EmptyRequest
        :return: an empty response
        :rtype: EmptyResponse
        """
        # Recover from Error if set (basic functionality)
        if self._controller_mode == ControlMode.ERROR:
            rospy.loginfo(f"Resetting from ERROR state to JOINTS [Default]")
            self._controller_mode = ControlMode.JOINTS
            self.preempted = False
        else:
            rospy.logwarn(f'Robot [{self.name}] not in ERROR state. Do Nothing.')

        return EmptyResponse()
    
    def update_description_cb(self, req: UpdateDescriptionRequest) -> UpdateDescriptionResponse: # pylint: disable=no-self-use
        """[summary]
        ROS Service callback:
        Updates the robot description if loaded into param

        :param req: an empty request
        :type req: EmptyRequest
        :return: an empty response
        :rtype: EmptyResponse
        """
        rospy.logwarn('TF update not implemented for this arm <IN DEV>')
        rospy.loginfo(f"req gripper: {req.gripper} | param: {req.param}")
        if req.gripper == '' or req.param == '':
            rospy.logerr(f"Inputs are None or Empty")
            return UpdateDescriptionResponse(success=False)
        
        gripper_link = None
        gripper = None

        # Preempt any motion prior to changing link structure
        if self.moving:
            self.preempt()

        # Read req param and only proceed if successful
        links, _, _, _ = URDFRobot.URDF_read_description(wait=False, param=req.param)

        if np.any(links):
            #Do Something
            # Using requested gripper, update control point
            gripper = req.gripper
            gripper_link = list(filter(lambda link: link.name == gripper, links))

        # DEBUGGING
        # rospy.loginfo(f"requested gripper: {gripper} | requested gripper link: {gripper_link}")
        # rospy.loginfo(f"Updated links:")
        # for link in links:
        #     rospy.loginfo(f"{link}")

        # Update robot tree if successful
        if np.any(links) and gripper_link != []: 
            # Remove the old dict of links
            self.link_dict.clear()

            # Clear current base link
            self._base_link = None

            # Sort and update new links and gripper links
            self._sort_links(links, gripper_link, True) 

            # Update control point
            self.gripper = gripper

            # Trigger backend reset
            self.backend_reset = True

            rospy.loginfo(f"Updated Links! New Control: {self.gripper}")
            return UpdateDescriptionResponse(success=True)
        else:
            if gripper_link == []: rospy.logwarn(f"Requested control tf [{req.gripper}] not found in tree")
            if links == None: rospy.logerr(f"No links found in description. Make sure {req.param} param is correct")
            return UpdateDescriptionResponse(success=False)

    def calibrate_transform_cb(self, req: CalibrateTransformRequest) -> CalibrateTransformResponse: # pylint: disable=no-self-use
        """[summary]
        ROS Service callback:
        Attempts to calibrate the location of a link if applicable
        NOTE: links that are associated to robot joints are ignored

        :param req: a request to calibrate a link, contains the transform and link name 
        :type req: CalibrateTransformRequest
        :return: a success bool, where True is if the link was found and applicable, then set; otherwise False
        :rtype: CalibrateTransformResponse
        """
        link_found = False

        rospy.loginfo(f"Got req for transform: {req.transform} | offset: {req.link_name}")

        # Early termination on input error
        if req.link_name == None or req.transform == Transform():
            rospy.logerr(f"Input values are None or Empty")
            return CalibrateTransformResponse(success=False)
        
        # Convert transform quaternion to rpy for updating Elementary Transform Sequence (ETS) of link
        # NOTE: the order is required to set correctly
        rpy = sm.UnitQuaternion(
                req.transform.rotation.w, [
                req.transform.rotation.x,
                req.transform.rotation.y,
                req.transform.rotation.z
            ]).rpy(order='zyx')

        # Update any transforms as requested on main robot (that is not a joint)
        # NOTE: joint tf's are to be immutable (as this is assumed the robot)
        # TODO: check if parent is base link 
        for link in self.links:
            # Update if found and applicable
            if link.name == req.link_name and not link.isjoint:
                rospy.loginfo(f"LINK -> {link.name} | PARENT: {link.parent_name} | BASE: {self.base_link}")
                # NOTE: the Elementary Transform Sequence (ETS) needs the orientation
                #       in (rpy) to be applied in required order. In this case, the
                #       order is 'zyx' (see above), therefore, apply in this order
                link.ets = rtb.ET.tx(req.transform.translation.x) \
                    * rtb.ET.ty(req.transform.translation.y) \
                    * rtb.ET.tz(req.transform.translation.z) \
                    * rtb.ET.Rz(rpy[2]) \
                    * rtb.ET.Ry(rpy[1]) \
                    * rtb.ET.Rx(rpy[0]) \
                
                link_found = True
                break

        # Re-run the collision overlap dictionary update for this robot as some links may have changed
        if link_found:
            self.characterise_collision_overlaps()

        rospy.loginfo(f"Transform Calibration Pipeline Completed.")
        return CalibrateTransformResponse(success=link_found)

    def set_cartesian_impedance_cb(  # pylint: disable=no-self-use
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
    
    def add_named_pose_in_frame_cb(self, req: AddNamedPoseInFrameRequest) -> AddNamedPoseInFrameResponse:
        """
        """
        named_poses = {}
        # Defaults to /home/qcr/.ros/configs/system_named_poses.yaml
        # config_file = self.config_path if not self.custom_configs else self.custom_configs[-1]
        config_file = '/home/qcr/armer_ws/src/armer_descriptions/data/custom/cgras_descriptions/config/named_poses.yaml'
        config_file = config_file.replace('.yaml', '_in_frame.yaml')
        try:
            config = yaml.load(open(config_file), Loader=yaml.SafeLoader)
            if config and 'named_poses' in config and config['named_poses'] != None:
                named_poses = config['named_poses']
        except IOError:
            rospy.logwarn(
                'Unable to locate configuration file: {}'.format(config_file))
            return AddNamedPoseInFrameResponse(success=False)            
           
        if req.name in named_poses and not req.overwrite:
            rospy.logerr('Named pose already exists.')
            return AddNamedPoseInFrameResponse(success=False)

        # TODO: transform into frame requested and save PoseStamped
        ## named PoseStamped position
        ee_pose = self.ets(start=self.base_link, end=self.gripper).eval(self.q.tolist())
        rospy.logerr(f'The self.base_link.name is: {self.base_link.name}')

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_link.name

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

        # TODO: do the transform...
        reference_frame_id = req.wrt_frame_id if req.wrt_frame_id != '' else '/base_link'
        tf = self.tf_listener.transformPose(
                    reference_frame_id,
                    pose_stamped,
                )

        # TODO: get real serialisation for PoseStamped to YAML...(use JSON!)
        yaml_posestamped = {}
        yaml_posestamped['frame_id'] = reference_frame_id
        yaml_posestamped['position'] = np.array([tf.pose.position.x, tf.pose.position.y, tf.pose.position.z]).tolist()
        yaml_posestamped['orientation'] = np.array([tf.pose.orientation.w, tf.pose.orientation.x, tf.pose.orientation.y, tf.pose.orientation.z]).tolist()

        named_poses[req.name] = yaml_posestamped

        self.__write_config('named_poses', named_poses, config_file)

        return AddNamedPoseInFrameResponse(success=True)

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

        self.named_poses[req.pose_name] = self.q.tolist()
        config_file = self.config_path if not self.custom_configs else self.custom_configs[-1]
        self.__write_config('named_poses', self.named_poses, config_file)

        return AddNamedPoseResponse(success=True)

    def remove_named_pose_cb(self, req: RemoveNamedPoseRequest) -> RemoveNamedPoseResponse:
        """
        ROS Service callback:
        Adds the current arm pose as a named pose and saves it to the host config

        :param req: The name of the pose
        :type req: RemoveNamedPoseRequest
        :return: True if the named pose was removed successfully otherwise false
        :rtype: RemoveNamedPoseResponse
        """
        if req.pose_name not in self.named_poses:
            rospy.logerr('Named pose does not exists.')
            return RemoveNamedPoseResponse(success=False)

        del self.named_poses[req.pose_name]
        config_file = self.config_path if not self.custom_configs else self.custom_configs[-1]
        self.__write_config('named_poses', self.named_poses, config_file)

        return RemoveNamedPoseResponse(success=True)

    def export_named_pose_config_cb(
            self,
            request: NamedPoseConfigRequest) -> NamedPoseConfigResponse:
        """[summary]
        Creates a config file containing the currently loaded named_poses

        :param request: [destination]
        :type request: NamedPoseConfigRequest
        :return: [bool]
        :rtype: NamedPoseConfigRequest
        """

        # Ensure the set of named_poses is up-to-date
        self.__load_config()

        # Write to provided config_path
        self.__write_config('named_poses', self.named_poses, request.config_path)
        return True

    def add_named_pose_config_cb(
            self,
            request: NamedPoseConfigRequest) -> NamedPoseConfigResponse:
        """[summary]

        :param request: [description]
        :type request: NamedPoseConfigRequest
        :return: [description]
        :rtype: NamedPoseConfigResponse
        """
        self.custom_configs.append(request.config_path)
        self.__load_config()
        return True

    def remove_named_pose_config_cb(
            self,
            request: NamedPoseConfigRequest) -> NamedPoseConfigResponse:
        """[summary]

        :param request: [description]
        :type request: NamedPoseConfigRequest
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
        return str(self.custom_configs)
    
    def set_pid(self, msg):
        """
        Sets the pid value from a callback
        Deprecated 2023-06-19.
        """
        self.Kp = None
        self.Ki = None
        self.Kd = None

    # --------------------------------------------------------------------- #
    # --------- Collision and Singularity Checking Methods ---------------- #
    # --------------------------------------------------------------------- #
    def add_collision_obj(self, obj: sg.Shape):
        """
        TODO: Implement a way to add to existing collision dictionary at runtime
        """
        pass
    
    def get_link_collision_dict(self) -> dict():
        """
        Returns a dictionary of all associated links (names) which lists their respective collision data
        To be used by high-level armer class for collision handling
        """
        return self.collision_dict

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
                rospy.logerr(f"Characterise Collision Overlaps -> gripper name is invalid: {self.gripper}")
                return False 
            
            # Error handling on empty lick dictionary (should never happen but just in case)
            if self.link_dict == dict() or self.link_dict == None:
                rospy.logerr(f"Characterise Collision Overlaps -> link dictionary is invalid: {self.link_dict}")
                return False
            
            # Error handling on collision object dict
            if self.collision_dict == dict() or self.collision_dict == None:
                rospy.logerr(f"Characterise Collision Overlaps -> collision dictionary is invalid: [{self.collision_dict}]")
                return False
            
            # Alternative Method (METHOD 2) that is getting the list in a faster iterative method
            self.overlapped_link_dict = dict([
                (link.name, self.get_links_in_collision(
                    target_link=link.name, 
                    check_list=self.collision_dict[link.name], 
                    ignore_list=[],
                    output_name_list=True)
                )
                for link in reversed(self.sorted_links)])

            # using json.dumps() to Pretty Print O(n) time complexity
            rospy.loginfo(f"Characterise Collision Overlaps per link: {json.dumps(self.overlapped_link_dict, indent=4)}")

            # Older method (METHOD 1)
            # # Iterate forwards starting at base of tree
            # # NOTE: self.links are resolved links from base to ee
            # #       self.total_links are based on a URDF read of the available links
            # for link in reversed(self.sorted_links):
            #     collision = True
            #     links_in_collision_list = []
            #     while collision:
            #         # print(f"CHECKING Link name: {link.name}")
            #         # Check link collision of current link with full robot
            #         col_link, collision = self.check_link_collision(
            #             target_link=link.name, 
            #             stop_link=self.base_link.name,
            #             check_list=self.collision_dict[link.name], 
            #             ignore_list=links_in_collision_list
            #         )

            #         if collision:    
            #             # print(f"INIT: collision found for {link.name} with {col_link.name}")
            #             links_in_collision_list.append(col_link.name)

            #     rospy.loginfo(f"Characterise Collision Overlaps -> link: {link.name} found links in collision: {links_in_collision_list}")
            #     self.overlapped_link_dict[link.name] = links_in_collision_list

        # Reached end in success
        return True
    
    def get_links_in_collision(self, target_link: str, ignore_list: list = [], check_list: list = [], output_name_list: bool = False):
        """
        An alternative method that returns a list of links in collision with target link.
        NOTE: ignore list used to ignore known overlapped collisions (i.e., neighboring link collisions)
        NOTE: check_list is a list of Shape objects to check against.
        """
        with Timer("NEW Get Link Collision", enabled=False):
            # rospy.loginfo(f"Target link requested is: {target_link}")
            
            # Handle invalid link name input
            if target_link == '' or target_link == None or not isinstance(target_link, str):
                rospy.logwarn(f"Self Collision Check -> Link name [{target_link}] is invalid.")
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
                for link in reversed(self.links) \
                if (link.name not in ignore_list) and (link.name != target_link) and (link.iscollided(obj, skip=True))
            ])
            
            # print(f"links: {[link.name for link in self.links]}")
            # print(f"Collision Keys: {list(check_dict.keys())}")     
            # print(f"Collision Values: {list(check_dict.values())}")    

            # Output list of collisions or name of links based on input bool
            if output_name_list:
                return list(check_dict.keys())
            else:
                return list(check_dict.values())
    
    def check_link_collision(self, target_link: str, stop_link: str, ignore_list: list = [], check_list: list = []):
        """
        This method is similar to roboticstoolbox.robot.Robot.iscollided
        NOTE: ignore list used to ignore known overlapped collisions (i.e., neighboring link collisions)
        """
        with Timer(name="OLD Check Link Collision", enabled=False):
            # rospy.loginfo(f"Target link requested is: {target_link}")
            # Handle invalid link name input
            if target_link == '' or target_link == None or not isinstance(target_link, str):
                rospy.logwarn(f"Self Collision Check -> Link name [{target_link}] is invalid.")
                return None, False

            # Handle invalid link name input
            if stop_link == '' or stop_link == None or not isinstance(stop_link, str):
                rospy.logwarn(f"Self Collision Check -> Search stop link name [{stop_link}] is invalid.")
                return None, False

            # Handle invalid name in links
            if stop_link not in self.collision_dict.keys():
                rospy.logwarn(f"Self Collision Check -> Stop Link name [{stop_link}] is not in [{self.collision_dict.keys()}]")
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
            for link in reversed(self.links):
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
                        # rospy.logerr(f"Self Collision Check -> Link that is collided: {link.name}")
                        return link, True
                    
                # Terminate at check stop link
                # NOTE: have this happen afterwards so we still inclusively check for this link
                if link.name == stop_link:
                    # rospy.logwarn(f"Self Collision Check -> Terminating iteration at {link.name}")
                    return None, False
                
            return None, False

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
        
    # --------------------------------------------------------------------- #
    # --------- Standard Methods ------------------------------------------ #
    # --------------------------------------------------------------------- #
    def close(self):
        """
        Closes the action servers associated with this robot
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

        # Warn and Reset
        if self.singularity_approached:
            rospy.logwarn(f"PREEMPTED: Approaching singularity (index: {self.manip_scalar}) --> please home to fix")
            self.singularity_approached = False

        if self.collision_approached:
            rospy.logwarn(f"PREEMPTED: Collision Found --> please home to fix")
            self.collision_approached = False

        # NOTE: put robot object into ERROR state as we have been preempted
        #       currently only the Home action resets this back to ControlMode.Joints
        # TODO: need to ensure this is checked for most control inputs (currently handled in Cartesian and Joint Velocity callbacks)
        # TODO: need to check other methods of reset if needed.
        self.preempted = True
        self._controller_mode = ControlMode.ERROR
        self.last_update = 0

    def preempt_tracking(self, *args: list) -> None:
        """
        Stops any current motion
        """
        # pylint: disable=unused-argument
        if self.executor:
            self.executor.abort()

        # Warn and Reset
        if self.singularity_approached:
            rospy.logwarn(f"PREEMPTED: Approaching singularity (index: {self.manip_scalar}) --> please home to fix")
            self.singularity_approached = False

        self.preempted = True
        self._controller_mode = ControlMode.JOINTS
        self.last_update = 0

    def preempt_other(self, *args: list) -> None:
        """
        Stops any current motion
        """
        # pylint: disable=unused-argument
        if self.executor:
            self.executor.abort()

        # Warn and Reset
        if self.singularity_approached:
            rospy.logwarn(f"PREEMPTED: Approaching singularity (index: {self.manip_scalar}) --> please home to fix")
            self.singularity_approached = False

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
        self.last_update = rospy.get_time()

    def pose_within_workspace(self, pose=Pose):
        # TODO: The workspace should be stored in param server!!!
        path = rospy.get_param('/robot_workspace', '')
        if path == '':
            # No workspace defined - assume infinite workspace
            # TODO: remove debug message
            rospy.logwarn("No WORKSPACE DEFINED")
            return True
        
        if os.path.isfile(path) == False:
            # In this case fail safe as a workspace definition had been attempted by user
            rospy.logerr(f"[{self.name}] WORKSPACE COULD NOT BE DEFINED! (Invalid Path) Please check the workspace.yaml file located at {path}")
            return False
        
        # TODO: The workspace should be stored in param server
        # TODO: The workspace should only update when requested rather than re-sourcing constantly
        with open(path, 'r') as handle:
            config = yaml.load(handle, Loader=yaml.SafeLoader)

        workspace = config['workspace'] if 'workspace' in config else None
        rospy.logdebug(f"Boundary--: {workspace}")

        if workspace == None:
            rospy.logerr(f"[{self.name}] WORKSPACE COULD NOT BE DEFINED! (Invalid Yaml) Please check the workspace.yaml file located at {path}")
            # In this case fail safe as a workspace definition had been attempted by user
            return False

        min_x = workspace[0]['min'][0]['x']
        min_y = workspace[0]['min'][0]['y']
        min_z = workspace[0]['min'][0]['z']

        max_x = workspace[1]['max'][0]['x']
        max_y = workspace[1]['max'][0]['y']
        max_z = workspace[1]['max'][0]['z']

        # Check that cartesian position of end-effector is within defined constraints. 
        # NOTE: the following bounds are based from the base_link which is the origin point. 
        # Assumed main constraint is z-axis plane (added a y-axis/End-Point) plane termination condition as well
        # NOTE: this is assuming Left-to-Right motion w.r.t Robot base_link
        if(pose.position.x <= min_x or pose.position.x >= max_x or \
            pose.position.y <= min_y or pose.position.y >= max_y or \
                pose.position.z <= min_z or pose.position.z >= max_z):

            rospy.logerr(f"[{self.name}] ROBOT **would** EXCEEDED DEFINED BOUNDARY!!!")
            rospy.logerr(f"[{self.name}] - Goal pose.position: {pose.position}")
            return False
        else:
            return True
        
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
        header.stamp = rospy.Time.now()

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

        try:
            # end-effector velocity
            T = jacob0 @ self.qd
        except:
            state.ee_velocity = TwistStamped()
            return state

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
            state.joint_torques = np.zeros(self.n)
        
        return state

    def test_guards(
        self,
        guards: Guards,
        start_time: float) -> int:

        triggered = 0

        if (guards.enabled & guards.GUARD_DURATION) == guards.GUARD_DURATION:
            triggered |= guards.GUARD_DURATION if rospy.get_time() - start_time > guards.duration else 0

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

    def step(self, dt: float = 0.01) -> None:  # pylint: disable=unused-argument
        """
        Updates the robot joints (robot.q) used in computing kinematics
        :param dt: the delta time since the last update, defaults to 0.01
        :type dt: float, optional
        """
        if self.readonly:
            return

        current_time = rospy.get_time()
        self.state = self.get_state()

        # PREEMPT motion on any detected state errors or singularity approach
        if self.state.errors != 0 \
            or self.check_singularity(self.q):
            # print(f"preempting...")
            self.preempt()

        # calculate joint velocities from desired cartesian velocity
        if self._controller_mode == ControlMode.CARTESIAN:
            if current_time - self.last_update > 0.1:
                self.e_v *= 0.9 if np.sum(np.absolute(self.e_v)
                                          ) >= 0.0001 else 0

                if np.all(self.e_v == 0):
                    self._controller_mode = ControlMode.JOINTS

            try:
                _, orientation = self.tf_listener.lookupTransform(
                    self.base_link.name,
                    self.e_v_frame,
                    rospy.Time(0)
                )
                
                U = UnitQuaternion([
                    orientation[-1],
                    *orientation[:3]
                ], norm=True, check=False).SE3()
                
                e_v = np.concatenate((
                (U.A @ np.concatenate((self.e_v[:3], [1]), axis=0))[:3],
                (U.A @ np.concatenate((self.e_v[3:], [1]), axis=0))[:3]
                ), axis=0)
                
                # Calculate error in base frame
                p = self.e_p.A[:3, 3] + e_v[:3] * dt                     # expected position
                Rq = UnitQuaternion.RPY(e_v[3:] * dt) * UnitQuaternion(self.e_p.R)
                
                T = SE3.Rt(SO3(Rq.R), p, check=False)   # expected pose
                Tactual = self.fkine(self.q, start=self.base_link, end=self.gripper) # actual pose
                
                e_rot = (SO3(T.R @ np.linalg.pinv(Tactual.R), check=False).rpy() + np.pi) % (2*np.pi) - np.pi
                error = np.concatenate((p - Tactual.t, e_rot), axis=0)
                
                e_v = e_v + error
                
                self.e_p = T
                            
                self.j_v = np.linalg.pinv(
                self.jacob0(self.q, end=self.gripper)) @ e_v
              
            except (tf.LookupException, tf2_ros.ExtrapolationException):
              rospy.logwarn('No valid transform found between %s and %s', self.base_link.name, self.e_v_frame)
              self.preempt()

        # apply desired joint velocity to robot
        # NOTE: this should be allowed to run even in ControlMode.ERROR to recover (i.e., Home action)
        if self.executor:
            self.j_v = self.executor.step(dt)

            # TODO: Remove?
            # - Need to validate if this is still required
            # - - IF it is required the rounding limit should be a param
            # if any(False if np.absolute(v) >= 0.000000001 else True for v in self.j_v):
            #     rospy.logerr(f"We are rounding a joint velocity in the executor...{self.j_v}")
            
            # bob = [v if np.absolute(v) >= 0.000000001 else 0 for v in self.j_v]
            # if np.any(self.j_v != bob):
            #     rospy.logerr(f"-- overwriting join velocities with rounded version...{bob}")
            #     self.j_v = bob
        else:
            # TODO: Remove?
            # - Need to validate if this is still required
            # - - IF it is required the rounding limit should be a param
            # if any(False if np.absolute(v) >= 0.0001 else True for v in self.j_v):
            #     rospy.logerr(f"We are rounding a joint velocity without executor...{self.j_v}")

            # bob = [v if np.absolute(v) >= 0.0001 else 0 for v in self.j_v]
            # if np.any(self.j_v != bob):
            #     rospy.logerr(f"-- overwriting join velocities with rounded version...{bob}")
            #     self.j_v = bob

            # Needed for preempting joint velocity control
            # - this is painfully hard coded...
            # -- should it be the sum or just an any(abs(value) >= min)?
            # -- should the else case be an array of zeros?
            if any(self.j_v) and current_time - self.last_update > 0.1:
                self.j_v = [v * 0.9 if np.absolute(v) > 0 else 0 for v in self.j_v]
            
        self.qd = self.j_v
        self.last_tick = current_time

        self.state_publisher.publish(self.state)

        self.event.set()

    def __load_config(self):
        """[summary]
        """
        self.named_poses = {}
        for config_path in self.custom_configs:
            try:
                config = yaml.load(open(config_path), Loader=yaml.SafeLoader)
                if config and 'named_poses' in config:
                    self.named_poses.update(config['named_poses'])
            except IOError:
                rospy.logwarn(
                    'Unable to locate configuration file: {}'.format(config_path))

        if os.path.exists(self.config_path):
            try:
                config = yaml.load(open(self.config_path),
                                   Loader=yaml.SafeLoader)
                if config and 'named_poses' in config:
                    self.named_poses.update(config['named_poses'])
            except IOError:
                pass

    def __write_config(self, key: str, value: Any, config_path: str=''):
        """[summary]

        :param key: [description]
        :type key: str
        :param value: [description]
        :type value: Any
        """
        if config_path == '':
            # Use default config_path
            # NOTE: the default config_path is /home/.ros/configs/system_named_poses.yaml
            config_path = self.config_path

        if not os.path.exists(os.path.dirname(config_path)):
            os.makedirs(os.path.dirname(config_path))

        config = {}

        try:
            with open(config_path) as handle:
                current = yaml.load(handle.read(), Loader=yaml.SafeLoader)

                if current:
                    config = current

        except IOError:
            pass

        config.update({key: value})

        with open(config_path, 'w') as handle:
            handle.write(yaml.dump(config))
