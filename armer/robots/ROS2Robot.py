"""
ROSRobot module defines the ROSRobot type

.. codeauthor:: Gavin Suddreys
"""
from armer.errors import ArmerError
import rclpy

from armer.robots import BaseRobot

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

from std_msgs.msg import Float64MultiArray

from armer_msgs.msg import ManipulatorState, JointVelocity, ServoStamped, Guards
from armer_msgs.action import GuardedVelocity, Home, MoveToJointPose, MoveToNamedPose, MoveToPose

# pylint: disable=too-many-instance-attributes

class ControlMode:
   JOINTS=1
   CARTESIAN=2

class ROS2Robot(BaseRobot):
    """
    The ROSRobot class wraps the rtb.ERobot implementing basic ROS functionality
    """

    def __init__(self, nh, *args, **kwargs):  # pylint: disable=unused-argument
        
        super().__init__(nh, *args, **kwargs)
        
        self.nh.create_subscription(
          JointState,
          self.joint_state_topic,
          self._state_cb,
          1
        )
        
        if not self.readonly:
            self.joint_publisher = self.nh.create_publisher(
              Float64MultiArray,
              self.joint_velocity_topic,
              1
            )


            # Publishers
            self.state_publisher = self.nh.create_publisher(
                ManipulatorState, '{}/state'.format(self.name.lower()), 1
            )
            self.cartesian_servo_publisher: self.nh.create_publisher(
                Bool, '{}/cartesian/servo/arrived'.format(self.name.lower()), 1
            )

            rclpy.action.ActionServer(
              self.nh,
              MoveToPose,
              '{}/cartesian/pose'.format(self.name.lower()),
              self.pose_cb
            )
            
            return
            
            # Services
            rospy.Service('{}/recover'.format(self.name.lower()),
                          Empty, self.recover_cb)
            rospy.Service('{}/stop'.format(self.name.lower()),
                          Empty, self.preempt)


            # Subscribers
            self.cartesian_velocity_subscriber: rospy.Subscriber = rospy.Subscriber(
                '{}/cartesian/velocity'.format(self.name.lower()
                                               ), TwistStamped, self.velocity_cb
            )
            self.joint_velocity_subscriber: rospy.Subscriber = rospy.Subscriber(
                '{}/joint/velocity'.format(self.name.lower()
                                           ), JointVelocity, self.joint_velocity_cb
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

    def close(self):
        """
        Closes the action servers associated with this robot
        """
        self.pose_server.need_to_terminate = True
        self.joint_pose_server.need_to_terminate = True
        self.named_pose_server.need_to_terminate = True

    def pose_cb(self, goal_handle) -> None:
        """
        ROS Action Server callback:
        Moves the end-effector to the
        cartesian pose indicated by goal

        :param goal: [description]
        :type goal: MoveToPoseGoal
        """
        result = super().pose_cb(goal_handle.request)
        
        if result:
          goal_handle.succeed()
        else:
          goal_handle.abort()

        return MoveToPose.Result(success=result)

    def joint_pose_cb(self, goal_handle) -> None:
        """
        ROS Action Server callback:
        Moves the arm the named pose indicated by goal

        :param goal: Goal message containing the name of
        the joint configuration to which the arm should move
        :type goal: MoveToNamedPoseGoal
        """
        result = super().joint_pose_cb(goal_handle.request)
        
        if result:
          goal_handle.succeed()
        else:
          goal_handle.abort()

        return MoveToJointPose.Result(success=result)

    def named_pose_cb(self, goal_handle) -> None:
        """
        ROS Action Server callback:
        Moves the arm the named pose indicated by goal

        :param goal: Goal message containing the name of
        the joint configuration to which the arm should move
        :type goal: MoveToNamedPoseGoal
        """
        try:
          result = super().named_pose_cb(goal_handle.request)
        except ArmerError:
          result = False
        
        if result:
          goal_handle.succeed()
        else:
          goal_handle.abort()

        return MoveToNamedPose.Result(success=result)
        

    def home_cb(self, goal_handle) -> None:
        """[summary]

        :param req: Empty request
        :type req: EmptyRequest
        :return: Empty response
        :rtype: EmptyResponse
        """
        result = super().home_cb(goal_handle.request)

        if result:
          goal_handle.succeed()
        else:
          goal_handle.abort()

        return Home.Result(success=result)

    def recover_cb(self, req): # pylint: disable=no-self-use
        """[summary]
        ROS Service callback:
        Invoke any available error recovery functions on the robot when an error occurs

        :param req: an empty request
        :type req: EmptyRequest
        :return: an empty response
        :rtype: EmptyResponse
        """
        pass

    def set_cartesian_impedance_cb(self, request):
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
        pass