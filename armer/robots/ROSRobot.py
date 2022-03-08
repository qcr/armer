"""
ROSRobot module defines the ROSRobot type

.. codeauthor:: Gavin Suddreys
"""
import os
import timeit

from typing import List, Any
from threading import Lock, Event
from armer.timer import Timer
import rospy
import actionlib
import tf
import roboticstoolbox as rtb
import spatialmath as sp
from spatialmath import SE3, SO3, UnitQuaternion, base
import numpy as np
import yaml

from armer.utils import ikine, mjtg

from std_msgs.msg import Header, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import TwistStamped, Twist, Vector3Stamped, QuaternionStamped
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from std_msgs.msg import Float64MultiArray

from armer_msgs.msg import ManipulatorState, JointVelocity, ServoStamped, Guards
from armer_msgs.msg import GuardedVelocityAction, GuardedVelocityGoal, GuardedVelocityResult
from armer_msgs.msg import MoveToJointPoseAction, MoveToJointPoseGoal, MoveToJointPoseResult
from armer_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseGoal, MoveToNamedPoseResult
from armer_msgs.msg import MoveToPoseAction, MoveToPoseGoal, MoveToPoseResult
from armer_msgs.msg import HomeAction, HomeGoal, HomeResult

from armer_msgs.srv import SetCartesianImpedance, \
    SetCartesianImpedanceRequest, \
    SetCartesianImpedanceResponse

from armer_msgs.srv import AddNamedPose, \
    AddNamedPoseRequest, \
    AddNamedPoseResponse

from armer_msgs.srv import AddNamedPoseConfig, \
    AddNamedPoseConfigRequest, \
    AddNamedPoseConfigResponse

from armer_msgs.srv import GetNamedPoseConfigs, \
    GetNamedPoseConfigsRequest, \
    GetNamedPoseConfigsResponse

from armer_msgs.srv import GetLinkName, \
    GetLinkNameRequest, \
    GetLinkNameResponse

from armer_msgs.srv import GetNamedPoses, \
    GetNamedPosesRequest, \
    GetNamedPosesResponse

from armer_msgs.srv import RemoveNamedPose, \
    RemoveNamedPoseRequest, \
    RemoveNamedPoseResponse

from armer_msgs.srv import RemoveNamedPoseConfig, \
    RemoveNamedPoseConfigRequest, \
    RemoveNamedPoseConfigResponse

# pylint: disable=too-many-instance-attributes

import tf2_ros

class ControlMode:
   JOINTS=1
   CARTESIAN=2

class ROSRobot(rtb.ERobot):
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
                 gripper=None,
                 frequency=None,
                 modified_qr=None,
                 Kp=None,
                 Ki=None,
                 Kd=None,
                 * args,
                 **kwargs):  # pylint: disable=unused-argument

        super().__init__(robot)
        self.__dict__.update(robot.__dict__)

        self.gripper = gripper if gripper else self.grippers[0].links[0].name
        self.name = name if name else self.name
        
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
            self.base = SE3(origin[:3]) @ SE3.RPY(origin[3:])

        self.frequency = frequency if frequency else rospy.get_param(joint_state_topic + '/frequency', 500)
        
        self.q = self.qr if hasattr(self, 'qr') else self.q # pylint: disable=no-member
        if modified_qr:
            self.qr = modified_qr
            self.q = modified_qr

        self.joint_states = None # Joint state message

        # Guards used to prevent multiple motion requests conflicting
        self.control_mode = ControlMode.JOINTS

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

        self.e_p = SE3(self.fkine(self.q, start=self.base_link, fast=True, end=self.gripper))

        self.Kp: float = Kp if Kp else 0.0
        self.Ki: float = Ki if Ki else 0.0
        self.Kd: float = Kd if Kd else 0.0

        self.last_update: float = 0
        self.last_tick: float = 0

        self.joint_subscriber = rospy.Subscriber(
            joint_state_topic if joint_state_topic else '/joint_states',
            JointState,
            self._state_cb
        )

        self.readonly = readonly

        self.joint_velocity_topic = joint_velocity_topic \
                if joint_velocity_topic \
                else '/joint_group_velocity_controller/command'

        if not self.readonly:
            self.joint_publisher = rospy.Publisher(
                self.joint_velocity_topic,
                Float64MultiArray,
                queue_size=1
            )

            # Create Transform Listener
            self.tf_listener = tf.TransformListener()

            self.config_path = config_path if config_path else os.path.join(
                os.getenv('HOME', '/root'),
                '.ros/configs/armer.yaml'
            )

            self.custom_configs: List[str] = []
            self.__load_config()

            # Services
            rospy.Service('{}/recover'.format(self.name.lower()),
                          Empty, self.recover_cb)
            rospy.Service('{}/stop'.format(self.name.lower()),
                          Empty, self.preempt)

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
            # -- Testing New Servo Subscriber
            self.cartesian_servo_publisher: rospy.Publisher = rospy.Publisher(
                '{}/cartesian/servo/arrived'.format(self.name.lower()
                    ), Bool, queue_size=1
            )
            self.cartesian_servo_subscriber: rospy.Subscriber = rospy.Subscriber(
                '{}/cartesian/servo'.format(self.name.lower()
                                            ), ServoStamped, self.servo_cb
            )

            # Action Servers
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

            self.home_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
                '{}/home'.format(self.name.lower()),
                HomeAction,
                execute_cb=self.home_cb,
                auto_start=False
            )
            self.home_server.register_preempt_callback(self.preempt)
            self.home_server.start()

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
            
            rospy.Service('{}/get_named_poses'.format(self.name.lower()), GetNamedPoses,
                          self.get_named_poses_cb)

            rospy.Service('{}/set_named_pose'.format(self.name.lower()), AddNamedPose,
                          self.add_named_pose_cb)
            rospy.Service('{}/remove_named_pose'.format(self.name.lower()), RemoveNamedPose,
                          self.remove_named_pose_cb)

            rospy.Service(
                '{}/add_named_pose_config'.format(self.name.lower()),
                AddNamedPoseConfig,
                self.add_named_pose_config_cb
            )
            rospy.Service(
                '{}/remove_named_pose_config'.format(self.name.lower()),
                RemoveNamedPoseConfig,
                self.remove_named_pose_config_cb
            )
            rospy.Service(
                '{}/get_named_pose_configs'.format(self.name.lower()),
                GetNamedPoseConfigs,
                self.get_named_pose_configs_cb
            )

            rospy.Subscriber(
                '{}/set_pid'.format(self.name.lower()),
                Float64MultiArray,
                self.set_pid
            )

    def set_pid(self, msg):
        self.Kp = msg.data[0]
        self.Ki = msg.data[1]
        self.Kd = msg.data[2]


    def close(self):
        """
        Closes the action servers associated with this robot
        """
        self.pose_server.need_to_terminate = True
        self.pose_servo_server.need_to_terminate = True
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

    def guarded_velocity_cb(self, msg: GuardedVelocityGoal) -> None:
        if self.moving:
            self.preempt()
        
        with self.lock:
            self.preempted = False
            
            start_time = timeit.default_timer()
            triggered = 0
            
            while not self.preempted:
                triggered = self.test_guards(msg.guards, start_time=start_time)

                if triggered != 0:
                    break

                self.__vel_move(msg.twist_stamped)
                rospy.sleep(0.01)

            self.velocity_server.set_succeeded(GuardedVelocityResult(triggered=True))

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
                goal_pose,
            )
            
            pose = goal_pose.pose
            
            solution = ikine(self, pose, q0=self.q, end=self.gripper)
            
            if self.__traj_move(solution.q, goal.speed if goal.speed else 0.2):
                self.pose_server.set_succeeded(MoveToPoseResult(success=True))
            else:
                self.pose_server.set_aborted(MoveToPoseResult(success=False))

    def servo_cb(self, msg) -> None:
        """
        ROS Servoing Subscriber Callback:
        Servos the end-effector to the cartesian pose given by msg
        
        :param msg: [description]
        :type msg: PoseStamped

        This callback makes use of the roboticstoolbox p_servo function
        to generate velocities at each timestep.
        """
        # rospy.loginfo(f"Received: {msg}")

        # Safely stop any current motion of the arm
        if self.moving:
            self.preempt()
        
        with self.lock:
            goal_pose = msg.pose
            goal_gain = msg.gain if msg.gain else 3
            goal_thresh = msg.threshold if msg.threshold else 0.005

            if msg.header.frame_id == '':
                msg.header.frame_id = self.base_link.name
            
            goal_pose_stamped = self.tf_listener.transformPose(
                self.base_link.name,
                PoseStamped(header=msg.header, pose=goal_pose)
            )
            pose = goal_pose_stamped.pose

            target = SE3(pose.position.x, pose.position.y, pose.position.z) * UnitQuaternion([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ]).SE3()

            arrived = False

            self.moving = True
            self.preempted = False

            velocities, arrived = rtb.p_servo(
                self.fkine(self.q, start=self.base_link, end=self.gripper),
                target,
                min(20, goal_gain),
                threshold=goal_thresh
            )

            self.j_v = np.linalg.pinv(
                self.jacobe(self.q)) @ velocities
            self.last_update = timeit.default_timer()

        self.cartesian_servo_publisher.publish(arrived)

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
            if self.__traj_move(np.array(goal.joints), goal.speed if goal.speed else 0.2):
                self.joint_pose_server.set_succeeded(
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
            if not goal.pose_name in self.named_poses:
                self.named_pose_server.set_aborted(
                    MoveToNamedPoseResult(success=False),
                    'Unknown named pose'
                )

            qd = np.array(self.named_poses[goal.pose_name])

            if self.__traj_move(qd, goal.speed if goal.speed else 0.2):
                self.named_pose_server.set_succeeded(
                    MoveToNamedPoseResult(success=True)
                )
            else:
                self.named_pose_server.set_aborted(
                    MoveToNamedPoseResult(success=False)
                )

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
            
            if self.__traj_move(qd, goal.speed if goal.speed else 0.2):
                self.home_server.set_succeeded(
                    HomeResult(success=True)
                )
            else:
                self.home_server.set_aborted(
                    HomeResult(success=False)
                )

    def recover_cb(self, req: EmptyRequest) -> EmptyResponse: # pylint: disable=no-self-use
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

    def preempt(self, *args: list) -> None:
        """
        Stops any current motion
        """
        # pylint: disable=unused-argument
        self.preempted = True
        self.control_mode = ControlMode.JOINTS
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

        if np.any(e_v - self.e_v) or self.control_mode == ControlMode.JOINTS:
            self.e_p = SE3(self.fkine(self.q, start=self.base_link, fast=True, end=self.gripper))

        self.e_v = e_v
        self.e_v_frame = twist_stamped.header.frame_id

        self.control_mode = ControlMode.CARTESIAN
        self.last_update = timeit.default_timer()

    def __traj_move(self, qd: np.array, max_speed=0.2, max_rot=0.5) -> bool:
        """[summary]

        :param traj: [description]
        :type traj: np.array
        :return: [description]
        :rtype: bool
        """

        if len(self.state.joint_poses) == 0:
          return False

        self.preempted = False
        self.moving = True

        t = 0

        # This is the average cartesian speed we want the robot to move at
        # NOTE: divided by 2 to make the max speed the approx. peak of the speed achieved
        ave_cart_speed = max_speed / 2
        # Frequency of operation - set by configuration file
        frequency = self.frequency
           
        # Calculate start and end pose linear distance to estimate the expected time
        current_ee_mat = self.fkine(self.q, start=self.base_link, fast=True, end=self.gripper)
        mid_ee_mat = self.fkine(qd - (qd - self.q) / 2, start=self.base_link, fast=True, end=self.gripper)
        end_ee_mat = self.fkine(qd, start=self.base_link, fast=True, end=self.gripper)
        
        current_ee_pose = current_ee_mat[:3, 3]
        mid_ee_pose = mid_ee_mat[:3, 3]
        end_ee_pose = end_ee_mat[:3, 3]
        
        # Estimation of time taken based on linear motion from current to end cartesian pose
        # We may require some optimisation of this given the curved nature of the actual ee trajectory
        D1 = np.sqrt((mid_ee_pose[0] - current_ee_pose[0])**2 +
            (mid_ee_pose[1] - current_ee_pose[1])**2 +
            (mid_ee_pose[2] - current_ee_pose[2])**2)
        
        D2 = np.sqrt((end_ee_pose[0] - mid_ee_pose[0])**2 +
            (end_ee_pose[1] - mid_ee_pose[1])**2 +
            (end_ee_pose[2] - mid_ee_pose[2])**2)

        linear_move_time = (D1 + D2) / ave_cart_speed

        current_ee_rot = current_ee_mat[:3,:3]
        end_ee_rot = end_ee_mat[:3,:3]

        angular_move_time = np.arccos((np.trace(np.transpose(end_ee_rot) @ current_ee_rot) - 1) / 2) / max_rot
        move_time = max(linear_move_time, angular_move_time)

        # Move time correction [currently un-used but requires optimisation]
        # Correction to account for error in curved motion
        move_time = move_time * 1

        # Obtain minimum jerk velocity profile of joints based on estimated end effector move time
        min_jerk_pos, min_jerk_vel = mjtg(self.q, qd, frequency, move_time)

        # Calculate time frequency - based on the max time required for trajectory and the frequency of operation
        time_freq_steps = int(move_time * frequency)

        #Time step initialise for trajectory
        time_step = 0
        cartesian_ee_vel_vect = []

        last_erro_jv = np.zeros(self.n)
        
        delta = 1/frequency
        last_time = rospy.Time.now().to_sec()
        
        # while move_time > 0.1 and t + delta <= move_time and time_step < time_freq_steps-1 and not self.preempted:
        while move_time > 0.1 and time_step < time_freq_steps-1 and not self.preempted:
            # Self termination if within goal space
            if np.all(np.fabs(qd - self.q) < 0.001):
                rospy.loginfo('Too close to goal, quitting movement...')
                break

            # Compute current state jacobian
            jacob0 = self.jacob0(self.q, fast=True, end=self.gripper)

            # Get current joint velocity and calculate current twist
            current_jv = self.state.joint_velocities #elf.j_v
            current_jp = self.state.joint_poses

            current_twist = jacob0 @ current_jv
            current_linear_vel = np.linalg.norm(current_twist[:3])
            cartesian_ee_vel_vect.append(current_linear_vel)

            # Calculate required joint velocity at this point in time based on minimum jerk
            req_jv = min_jerk_vel[time_step]
            req_jp = min_jerk_pos[time_step]
            
            # Calculate error in joint velocities based on current and expected
            erro_jv = req_jv - current_jv
            erro_jp = req_jp - current_jp

            if np.any(np.max(np.fabs(erro_jp)) > 0.5):
                rospy.logerr('Exceeded delta joint position max')
                self.preempt()
                break

            # Calculate corrected error based on error above
            corr_jv = req_jv + (erro_jv * self.Kp) + (erro_jp * self.Ki) + (((erro_jv - last_erro_jv) / delta) * self.Kd)
            last_erro_jv = erro_jv
            
            # Update of new expected joint velocities for low level controllers
            self.event.clear()
            self.j_v = corr_jv
            self.last_update = timeit.default_timer()
            self.event.wait()

            # Increment time step(s)
            time_step += 1  #Step value from calculated trajectory (minimum jerk)
            delta = rospy.Time.now().to_sec() - last_time
            last_time = rospy.Time.now().to_sec()
            t += delta  
        
        self.j_v = np.zeros(self.n)
        rospy.logwarn(f"End of Traj Move\n-> [ move time: {move_time} | time taken: {t} ]\
            \n-> End joint Error: {abs(min_jerk_pos[-1]-self.state.joint_poses) if min_jerk_pos else None}\
            \n-> Time step: {time_step} | time steps req: {time_freq_steps}")
        self.moving = False
        result = not self.preempted
        self.preempted = False
        return result

    def get_state(self) -> ManipulatorState:
        """
        Generates a ManipulatorState message for the robot

        :return: ManipulatorState message describing the current state of the robot
        :rtype: ManipulatorState
        """
        jacob0 = self.jacob0(self.q, fast=True, end=self.gripper)
        
        ## end-effector position
        ee_pose = self.fkine(self.q, start=self.base_link, end=self.gripper, fast=True)
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
        ee_rot = sp.UnitQuaternion(rotation)

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
            state.joint_torques = np.zeros(self.n)
        
        return state

    def test_guards(
        self,
        guards: Guards,
        start_time: float) -> int:

        triggered = 0

        if (guards.enabled & guards.GUARD_DURATION) == guards.GUARD_DURATION:
            triggered |= guards.GUARD_DURATION if timeit.default_timer() - start_time > guards.duration else 0

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

        self.named_poses[req.pose_name] = self.q.tolist()
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

        current_time = timeit.default_timer()
        self.state = self.get_state()

        if self.state.errors != 0:
            self.preempt()

        # calculate joint velocities from desired cartesian velocity
        if self.control_mode == ControlMode.CARTESIAN:
            if current_time - self.last_update > 0.1:
                self.e_v *= 0.9 if np.sum(np.absolute(self.e_v)
                                          ) >= 0.0001 else 0

                if np.all(self.e_v == 0):
                    self.control_mode = ControlMode.JOINTS

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
              Tactual = SE3(self.fkine(self.q, start=self.base_link, fast=True, end=self.gripper), check=False) # actual pose
              
              e_rot = (SO3(T.R @ np.linalg.pinv(Tactual.R), check=False).rpy() + np.pi) % (2*np.pi) - np.pi
              error = np.concatenate((p - Tactual.t, e_rot), axis=0)
              
              e_v = e_v + error
              
              self.e_p = T
                            
              self.j_v = np.linalg.pinv(
                self.jacob0(self.q, fast=True, end=self.gripper)) @ e_v
              
            except (tf.LookupException, tf2_ros.ExtrapolationException):
              rospy.logwarn('No valid transform found between %s and %s', self.base_link.name, self.e_v_frame)
              self.preempt()

        # apply desired joint velocity to robot
        if any(self.j_v):
            if current_time - self.last_update > 0.1:
                self.j_v *= 0.9 if np.sum(np.absolute(self.j_v)
                                          ) >= 0.0001 else 0
            
        self.qd = self.j_v
        self.last_tick = current_time

        self.state_publisher.publish(self.state)

        self.event.set()

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
