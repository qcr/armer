"""
ROSRobot module defines the ROSRobot type

.. codeauthor:: Gavin Suddreys
.. codeauthor:: Dasun Gunasinghe
"""
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
from armer.trajectory import TrajectoryExecutor
from armer.utils import ikine, mjtg
from armer.errors import ArmerError

# pylint: disable=too-many-instance-attributes

from armer_msgs.msg import ManipulatorState, JointVelocity, ServoStamped, Guards
from armer_msgs.action import GuardedVelocity, Home, MoveToJointPose, MoveToNamedPose, MoveToPose
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped
from std_msgs.msg import Header, Float64MultiArray, Bool
from sensor_msgs.msg import JointState

class ControlMode:
   JOINTS=1
   CARTESIAN=2

class ROSRobot(URDFRobot):
    """
    The ROSRobot class wraps the rtb.ERobot implementing basic ROS functionality
    """

    def __init__(self,
                 nh,
                 name: str = None,
                 joint_state_topic: str = None,
                 joint_velocity_topic: str = None,
                 origin=None,
                 config_path=None,
                 readonly=False,
                 frequency=None,
                 modified_qr=None,
                 singularity_thresh=0.02,
                 * args,
                 **kwargs):  # pylint: disable=unused-argument
        
        super().__init__(nh, *args, **kwargs)
        
        self.name = name if name else self.name
        
        self.joint_state_topic = joint_state_topic \
                if joint_state_topic \
                else '/joint_states'

        self.joint_velocity_topic = joint_velocity_topic \
                if joint_velocity_topic \
                else '/joint_group_velocity_controller/command'

        self.config_path = config_path if config_path else os.path.join(
            os.getenv('HOME', '/root'),
            '.ros/configs/armer.yaml'
        )

        if not hasattr(self, 'gripper'):
          self.gripper = self.grippers[0].name
          
        sorted_links=[]
        #sort links by parents starting from gripper
        link=self.link_dict[self.gripper]   
        while link is not None:
            sorted_links.append(link)
            link=link.parent
        sorted_links.reverse()

        self.joint_indexes = []
        self.joint_names = list(map(lambda link: link._joint_name, filter(lambda link: link.isjoint, sorted_links)))
        
        if origin:
            self.base = sm.SE3(origin[:3]) @ sm.SE3.RPY(origin[3:])

        self.frequency = frequency if frequency else self.get_parameter(
          f'{self.joint_state_topic}/frequency', 500
        )
        
        self.q = self.qr if hasattr(self, 'qr') else self.q # pylint: disable=no-member
        if modified_qr:
            self.qr = modified_qr
            self.q = modified_qr

        self.joint_states = None # Joint state message

        # Guards used to prevent multiple motion requests conflicting
        self._controller_mode = ControlMode.JOINTS

        self.moving: bool = False
        self.last_moving: bool = False

        self.preempted: bool = False

        self.lock: Lock = Lock()
        self.event: Event = Event()

        # Arm state property
        self.state: ManipulatorState = ManipulatorState()

        self.e_v_frame: str = None # Expected cartesian velocity

        self.e_v: np.array = np.zeros(shape=(6,))  # cartesian motion
        self.j_v: np.array = np.zeros(
            shape=(len(self.q),)
        )  # expected joint velocity

        self.e_p = self.fkine(self.q, start=self.base_link, end=self.gripper)

        self.last_update: float = 0
        self.last_tick: float = 0

        self.executor = None
        
        self.traj_generator = mjtg
        self.ik_solver = ikine

        self.tf_buffer = tf2_ros.Buffer()
        
        if hasattr(self.nh, 'get_clock'):
          self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self.nh)
        else:
          self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer)

        self.readonly = readonly

        self.custom_configs: List[str] = []

        # Singularity index threshold (0 is a sigularity)
        # NOTE: this is a tested value and may require configuration (i.e., speed of robot)
        self.logger(f"[INIT] Singularity Scalar Threshold set to: {singularity_thresh}")
        self.singularity_thresh = singularity_thresh 
        self.manip_scalar = None
        self.singularity_approached = False

        self.__load_config()

        #### --- ROS SETUP --- ###
        self.nh.create_subscription(
          JointState,
          self.joint_state_topic,
          self._state_cb,
          1
        )
        
        if not self.readonly:
            # Publishers
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
            self.cartesian_servo_publisher: self.nh.create_publisher(
                Bool, 
                '{}/cartesian/servo/arrived'.format(self.name.lower()), 
                1
            )

        #     rclpy.action.ActionServer(
        #       self.nh,
        #       MoveToPose,
        #       '{}/cartesian/pose'.format(self.name.lower()),
        #       self.pose_cb
        #     )
            
        #     return
            
        #     # Services
        #     rospy.Service('{}/recover'.format(self.name.lower()),
        #                   Empty, self.recover_cb)
        #     rospy.Service('{}/stop'.format(self.name.lower()),
        #                   Empty, self.preempt)


        #     # Subscribers
        #     self.cartesian_velocity_subscriber: rospy.Subscriber = rospy.Subscriber(
        #         '{}/cartesian/velocity'.format(self.name.lower()
        #                                        ), TwistStamped, self.velocity_cb
        #     )
        #     self.joint_velocity_subscriber: rospy.Subscriber = rospy.Subscriber(
        #         '{}/joint/velocity'.format(self.name.lower()
        #                                    ), JointVelocity, self.joint_velocity_cb
        #     )
        #     self.cartesian_servo_subscriber: rospy.Subscriber = rospy.Subscriber(
        #         '{}/cartesian/servo'.format(self.name.lower()
        #                                     ), ServoStamped, self.servo_cb
        #     )

        #     # Action Servers
        #     self.velocity_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
        #         '{}/cartesian/guarded_velocity'.format(self.name.lower()),
        #         GuardedVelocityAction,
        #         execute_cb=self.guarded_velocity_cb,
        #         auto_start=False
        #     )
        #     self.velocity_server.register_preempt_callback(self.preempt)
        #     self.velocity_server.start()

        #     self.pose_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
        #         '{}/cartesian/pose'.format(self.name.lower()),
        #         MoveToPoseAction,
        #         execute_cb=self.pose_cb,
        #         auto_start=False
        #     )
        #     self.pose_server.register_preempt_callback(self.preempt)
        #     self.pose_server.start()

        #     self.joint_pose_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
        #         '{}/joint/pose'.format(self.name.lower()),
        #         MoveToJointPoseAction,
        #         execute_cb=self.joint_pose_cb,
        #         auto_start=False
        #     )
        #     self.joint_pose_server.register_preempt_callback(self.preempt)
        #     self.joint_pose_server.start()

        #     self.named_pose_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
        #         '{}/joint/named'.format(self.name.lower()),
        #         MoveToNamedPoseAction,
        #         execute_cb=self.named_pose_cb,
        #         auto_start=False
        #     )
        #     self.named_pose_server.register_preempt_callback(self.preempt)
        #     self.named_pose_server.start()

        #     self.home_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
        #         '{}/home'.format(self.name.lower()),
        #         HomeAction,
        #         execute_cb=self.home_cb,
        #         auto_start=False
        #     )
        #     self.home_server.register_preempt_callback(self.preempt)
        #     self.home_server.start()

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
            
        #     rospy.Service('{}/get_named_poses'.format(self.name.lower()), GetNamedPoses,
        #                   self.get_named_poses_cb)

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

        #     rospy.Subscriber(
        #         '{}/set_pid'.format(self.name.lower()),
        #         Float64MultiArray,
        #         self.set_pid
        #     )

    def close(self):
        """
        Closes any resources associated with this robot
        """
        self.pose_server.need_to_terminate = True
        self.joint_pose_server.need_to_terminate = True
        self.named_pose_server.need_to_terminate = True

    def _state_cb(self, msg):
        if not self.joint_indexes:
            for joint_name in self.joint_names:
                self.joint_indexes.append(msg.name.index(joint_name))
        
        self.q = np.array(msg.position)[self.joint_indexes] if len(msg.position) == self.n else np.zeros(self.n)
        self.joint_states = msg
        
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
            self.preempted = False
            self.__vel_move(msg)

    def guarded_velocity_cb(self, msg) -> None:
        if self.moving:
            self.preempt()
        
        with self.lock:
            self.preempted = False
            
            start_time = self.get_time()
            triggered = 0
            
            while not self.preempted:
                triggered = self.test_guards(msg.guards, start_time=start_time)

                if triggered != 0:
                    break

                self.__vel_move(msg.twist_stamped)
                time.sleep(0.01)

        return True

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
            self.last_update = self.get_time()

    def servo_cb(self, msg: ServoStamped) -> None:
        """
        ROS Servoing Subscriber Callback:
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
            goal_pose = msg.pose
            goal_gain = msg.gain if msg.gain else 3
            goal_thresh = msg.threshold if msg.threshold else 0.005

            if msg.header.frame_id == '':
                msg.header.frame_id = self.base_link.name
            
            goal_pose_stamped = self.transform(
                PoseStamped(header=msg.header, pose=goal_pose),
                self.base_link.name
            )
            
            pose = goal_pose_stamped.pose

            target = sm.SE3(pose.position.x, pose.position.y, pose.position.z) * sm.UnitQuaternion([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ]).sm.SE3()

            arrived = False

            self.moving = True
            self.preempted = False

            velocities, arrived = rtb.p_servo(
                self.ets(start=self.base_link, end=self.gripper).eval(self.q),
                target,
                min(20, goal_gain),
                threshold=goal_thresh
            )

            self.j_v = np.linalg.pinv(
                self.jacobe(self.q)) @ velocities
            self.last_update = self.get_time()

        self.cartesian_servo_publisher.publish(arrived)

    def pose_cb(self, goal):
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

            goal_pose = self.transform(goal_pose, self.base_link.name)
            
            pose = goal_pose.pose
            
            solution = ikine(self, pose, q0=self.q, end=self.gripper)
            
            self.executor = TrajectoryExecutor(
              self,
              self.traj_generator(self, solution.q, goal.speed if goal.speed else 0.2)
            )

            while not self.executor.is_finished():
              time.sleep(0.01)

            result = self.executor.is_succeeded()

            self.executor = None
            self.moving = False

        return result

    def joint_pose_cb(self, goal) -> None:
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

            self.executor = TrajectoryExecutor(
              self,
              self.traj_generator(self, np.array(goal.joints), goal.speed if goal.speed else 0.2)
            )

            while not self.executor.is_finished():
              time.sleep(0.01)

            result = self.executor.is_succeeded()
            
            self.executor = None
            self.moving = False

        return result

    def named_pose_cb(self, goal, result) -> None:
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
              raise ArmerError('Unknown named pose')

            qd = np.array(self.named_poses[goal.pose_name])

            self.executor = TrajectoryExecutor(
                self,
                self.traj_generator(self, qd, goal.speed if goal.speed else 0.2)
            )

            while not self.executor.is_finished():
                self.sleep(0.01)

            result = self.executor.is_succeeded()
            
            self.executor = None
            self.moving = False

        return result

    def home_cb(self, goal) -> None:
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
            
            self.executor = TrajectoryExecutor(
                self,
                self.traj_generator(self, qd, goal.speed if goal.speed else 0.2)
            )

            while not self.executor.is_finished():
              time.sleep(0.01)

            result = self.executor.is_succeeded()
                
            self.executor = None
            self.moving = False

        return result

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

    def get_named_poses_cb(self, req):
        """
        ROS Service callback:
        Retrieves the list of named poses available to the arm

        :param req: An empty request
        :type req: GetNamesListRequest
        :return: The list of named poses available for the arm
        :rtype: GetNamesListResponse
        """
        raise NotImplementedError()
        
    def add_named_pose_cb(self, req):
        """
        ROS Service callback:
        Adds the current arm pose as a named pose and saves it to the host config

        :param req: The name of the pose as well as whether to overwrite if the pose already exists
        :type req: AddNamedPoseRequest
        :return: True if the named pose was written successfully otherwise false
        :rtype: AddNamedPoseResponse
        """
        raise NotImplementedError()

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
            return True
        else:
            self.singularity_approached = False
            return False

    def step(self, dt: float = 0.01) -> None:  # pylint: disable=unused-argument
        """
        Updates the robot joints (robot.q) used in computing kinematics
        :param dt: the delta time since the last update, defaults to 0.01
        :type dt: float, optional
        """
        if self.readonly:
            return

        current_time = self.get_time()
        self.state = self.get_state()

        # PREEMPT motion on any detected state errors or singularity approach
        if self.state.errors != 0 or self.check_singularity(self.q):
            self.preempt()

        # calculate joint velocities from desired cartesian velocity
        if self._controller_mode == ControlMode.CARTESIAN:
            if current_time - self.last_update > 0.1:
                self.e_v *= 0.9 if np.sum(np.absolute(self.e_v)
                                          ) >= 0.0001 else 0

                if np.all(self.e_v == 0):
                    self._controller_mode = ControlMode.JOINTS

            try:
              print(f"base_link name: {self.base_link.name}")
              print(f"e_v_frame name: {self.e_v_frame}")
              print(f"get_time: {self.get_time(False)}")
              _, orientation = self.tf_buffer.lookup_transform(
                  self.base_link.name,
                  self.e_v_frame,
                  self.get_time(False)
              )
              
              U = sm.UnitQuaternion([
                  orientation[-1],
                  *orientation[:3]
              ], norm=True, check=False).sm.SE3()
              
              e_v = np.concatenate((
                (U.A @ np.concatenate((self.e_v[:3], [1]), axis=0))[:3],
                (U.A @ np.concatenate((self.e_v[3:], [1]), axis=0))[:3]
              ), axis=0)
              
              # Calculate error in base frame
              p = self.e_p.A[:3, 3] + e_v[:3] * dt                     # expected position
              Rq = sm.UnitQuaternion.RPY(e_v[3:] * dt) * sm.UnitQuaternion(self.e_p.R)
              
              T = sm.SE3.Rt(sm.SO3(Rq.R), p, check=False)   # expected pose
              Tactual = self.fkine(self.q, start=self.base_link, end=self.gripper) # actual pose
              
              e_rot = (sm.SO3(T.R @ np.linalg.pinv(Tactual.R), check=False).rpy() + np.pi) % (2*np.pi) - np.pi
              error = np.concatenate((p - Tactual.t, e_rot), axis=0)
              
              e_v = e_v + error
              
              self.e_p = T
                            
              self.j_v = np.linalg.pinv(
                self.jacob0(self.q, end=self.gripper)) @ e_v
              
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
              self.logger('No valid transform found between {} and {}'.format(self.base_link.name, self.e_v_frame), 'warn')
              self.preempt()

        # apply desired joint velocity to robot
        if self.executor:
          self.j_v = self.executor.step(dt)  
        else:
            # Needed for preempting joint velocity control
            if any(self.j_v) and current_time - self.last_update > 0.1:
                self.j_v *= 0.9 if np.sum(np.absolute(self.j_v)) >= 0.0001 else 0
            
            
        self.qd = self.j_v
        self.last_tick = current_time

        self.state_publisher.publish(self.state)

        self.event.set()

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
