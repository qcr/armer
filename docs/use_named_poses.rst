Using Named Poses
=====================

Armer provides the ability to save and load poses for any arm. 

Adding a Named Pose
---------------------

Simply navigate a manipulator into any pose by any means and call the service ``/arm/set_named_pose``. The pose is saved locally to a config file in YAML format and automatically loaded each time Armer is launched.

The service can be summoned via Python:

.. code-block:: python

    from armer_msgs.srv import AddNamedPose
    import rospy

    rospy.init_node('armer_example', disable_signals=True)
    add_pose_service = rospy.ServiceProxy('/arm/set_named_pose', AddNamedPose)
    
    named_pose="my_pose"
    add_pose_service(named_pose, True)


Or via bash command line:

.. code-block:: bash

    rosservice call /arm/set_named_pose "pose_name: {DESIRED_POSE_NAME}"

The pose is now saved and the robot can navigate to the saved pose by making a request to the action server. 

Moving to a Named Pose
------------------------

The Armer interface uses a ROS action server to listen for requests to move to a named pose.

Requests to the ROS action server can be made by creating a ROS action client in Python. This client is used to send requests to the action server.

This example shows a request to move to named pose `my_pose`:

.. code-block:: python

    from armer_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseGoal 
    import actionlib
    import rospy

    rospy.init_node('armer_example', disable_signals=True)

    named_pose="my_pose"

    servo_cli = actionlib.SimpleActionClient('/arm/joint/named', MoveToNamedPoseAction)
    servo_cli.wait_for_server()

    goal = MoveToNamedPoseGoal()
    goal.pose_name=named_pose

    # Send goal and wait for it to finish
    servo_cli.send_goal(goal)
    servo_cli.wait_for_result()
    servo_cli.get_result()

Removing a Named Pose
-----------------------

Named poses can be removed via the ``/arm/remove_named_pose`` service. This can be called via Python:

.. code-block:: python

    from armer_msgs.srv import RemoveNamedPose
    import rospy

    rospy.init_node('armer_example', disable_signals=True)
    remove_pose_service = rospy.ServiceProxy('/arm/remove_named_pose', RemoveNamedPose)
    
    named_pose="my_pose"
    remove_pose_service(named_pose)    

Or bash: 

.. code-block:: bash

    rosservice call /arm/remove_named_pose "pose_name: {POSE_TO_REMOVE}"

Getting Saved Named Poses
--------------------------

To see a list of poses are saved, use the ``/arm/get_named_poses`` service.

Via Python:

.. code-block:: python

    from armer_msgs.srv import GetNamedPoses
    import rospy

    rospy.init_node('armer_example', disable_signals=True)
    get_poses_service = rospy.ServiceProxy('/arm/get_named_poses', GetNamedPoses)

    get_poses_service()    

Bash: 

.. code-block:: bash

    rosservice call /arm/get_named_poses


Loading Named Poses from Config files
---------------------------------------

To load configs from a YAML other than the default Armer config, the ``/arm/add_named_pose_config`` service can be used.

It can be summoned via Python or Bash.

Python:

.. code-block:: python

    from armer_msgs.srv import AddNamedPoseConfig
    import rospy

    rospy.init_node('armer_example', disable_signals=True)
    save_config_service = rospy.ServiceProxy('/arm/add_named_pose_config', AddNamedPoseConfig)
    
    config_path="/home/user/saved_poses.yaml"
    save_config_service(config_path)    

Bash: 

.. code-block:: bash

    rosservice call /arm/add_named_pose_config "config_path: {PATH_TO_CONFIG.yaml}"


Removing Named Poses from Config files
-----------------------------------------

To remove the config poses, the ``/arm/remove_named_pose_config`` service can be called. 

Via Python:

.. code-block:: python

    from armer_msgs.srv import RemoveNamedPoseConfig
    import rospy

    rospy.init_node('armer_example', disable_signals=True)
    remove_config_service = rospy.ServiceProxy('/arm/remove_named_pose_config', RemoveNamedPoseConfig)
    
    config_path="/home/user/saved_poses.yaml"
    remove_config_service(config_path)    

Bash: 

.. code-block:: bash

    rosservice call /arm/remove_named_pose_config "config_path: {PATH_TO_CONFIG.yaml}"

