Setting Joint Positions
====================================

The Armer interface contains a ROS action server which listens for requests to set joint positions.

The user can use a ROS action client to set the joint positions of any or all of the joints on a manipulator.


Via Python
-----------------

Requests to the ROS action server can be made by creating a ROS action client in Python. This client is used to send requests to the action server.

This example shows the setting of joints 1, 4, 5, and 6 to 0 radians, and joints 2 and 3 to pi/2 and pi/4 radians respectively.

.. code-block:: python

    from math import pi
    from armer_msgs.msg import MoveToJointPoseAction, MoveToJointPoseGoal 
    import actionlib
    import rospy

    rospy.init_node('armer_example', disable_signals=True)
    servo_cli = actionlib.SimpleActionClient('/arm/joint/pose', MoveToJointPoseAction)
    servo_cli.wait_for_server()

    goal = MoveToJointPoseGoal()
    goal.joints=[0, -pi/2, pi/4, 0, 0, 0]
    servo_cli.send_goal(goal)
    servo_cli.wait_for_result()
    servo_cli.get_result()

After the imports, a ROS client is created and following this, a request message is created with the joint goals and sent to the action server.