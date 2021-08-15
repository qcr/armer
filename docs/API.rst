API
==========
Subscribed Topics
-------------------

* **/arm/cartesian/velocity** `geometry_msgs/TwistStamped <https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html>`_
    Moves the end-effector in cartesian space w.r.t. the target frame_id (base frame if no frame_id is set).

* **/arm/joint/velocity** `armer_msgs/JointVelocity <https://github.com/qcr/armer_msgs/blob/main/msg/JointVelocity.html>`_
    Moves the joints of the manipulator at the requested velocity.

Published Topics
-----------------

* **/arm/state**  `armer_msgs/ManipulatorState <https://github.com/qcr/armer_msgs/blob/main/msg/ManipulatorState.msg>`_
    Provides information on the current state of the manipulator including the pose of the end-effector w.r.t. to the base link, whether the manipulator is experiencing a cartesian contact and collision as a bit-wised error state flag.

Services
-----------

* **/arm/home** `std_srvs/Empty <http://docs.ros.org/noetic/api/std_srvs/html/srv/Empty.html>`_
    Moves the robot back to its initial ready pose.

* **/arm/recover** `std_srvs/Empty <http://docs.ros.org/noetic/api/std_srvs/html/srv/Empty.html>`_
    Recovers from collision or limit violation error states that will put the robot into a non-operable state.

* **/arm/stop** `std_srvs/Empty <http://docs.ros.org/noetic/api/std_srvs/html/srv/Empty.html>`_
    Stops the current motion of the current.

* **/arm/get_named_poses** `armer_msgs/GetNamesList <https://github.com/qcr/armer_msgs/blob/main/srv/GetNamesList.srv>`_
    Gets a list of currently stored named poses (includes both moveit and driver stored named poses).

* **/arm/set_named_pose** `armer_msgs/SetNamedPose <https://github.com/qcr/armer_msgs/blob/main/srv/SetNamedPose.srv>`_
    Saves the current joint configuration of the robot with the provided pose name.

* **/arm/remove_named_pose** `armer_msgs/RemoveNamedPose <https://github.com/qcr/armer_msgs/blob/main/srv/RemoveNamedPose.srv>`_
    Removes the joint configuration of the provided pose name.

* **/arm/set_cartesian_impedance** `armer_msgs/SetCartesianImpedance <https://github.com/qcr/armer_msgs/blob/main/srv/SetCartesianImpedance.srv>`_
    Adjusts the impedance of the end-effector position in cartesian space.

* **/arm/add_named_pose_config** `armer_msgs/SetNamedPoseConfig <https://github.com/qcr/armer_msgs/blob/main/srv/SetNamedPoseConfig.srv>`_
    Instructs the driver to load named poses stored in the indicated config file.

* **/arm/get_named_pose_configs** `armer_msgs/GetNamedPoseConfigs <https://github.com/qcr/armer_msgs/blob/main/srv/GetNamedPoseConfigs.srv>`_
    Gets the list of config files to check for named poses.

* **/arm/remove_named_pose_config** `armer_msgs/SetNamedPoseConfig <https://github.com/qcr/armer_msgs/blob/main/srv/SetNamedPoseConfig.srv>`_
    Instructs the driver to remove named poses stored in the indicated config file.


Action API
-------------

* **/arm/cartesian/pose** `armer_msgs/MoveToPose.action <https://github.com/qcr/armer_msgs/blob/main/action/MoveToPose.action>`_
    Moves the end-effector to the requested goal pose w.r.t. the indicated frame id.

* **/arm/cartesian/servo_pose** `armer_msgs/ServoToPose.action <https://github.com/qcr/armer_msgs/blob/main/action/ServoToPose.action>`_
    Servos the end-effector to the requested goal pose with real time object avoidance.

* **/arm/joint/named** `armer_msgs/MoveToNamedPose.action <https://github.com/qcr/armer_msgs/blob/main/action/MoveToNamedPose.action>`_
    Moves the end-effector to a pre-defined joint configuration.

* **/arm/joint/pose** `armer_msgs/MoveToJointPoseAction <https://github.com/qcr/armer_msgs/blob/main/action/MoveToJointPose.action>`_
    Moves the joints of the robot to the indicated positions (radians).