Setting Cartesian Positions
====================================

The Armer interface contains a ROS action server which listens for requests to set a Cartesian position.

The user can use a ROS action client to set the Cartesian position of the end effector.


Via Python
-----------------

Requests to the ROS action server can be made by creating a ROS action client in Python. This client is used to send requests to the action server.

This example shows a request to the action server to move the arm so the end effector will be located at 0.300 meters in the x direction, 0.200 meters in the y direction and 0.290 in the z direction with a orientation of -1.00 in the x axis.

.. code-block:: python

    import rospy
    import actionlib
    from armer_msgs.msg import MoveToPoseAction, MoveToPoseGoal
    from geometry_msgs.msg import PoseStamped
    import rospy

    rospy.init_node('armer_example', disable_signals=True)
    pose_cli = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
    pose_cli.wait_for_server()

    target = PoseStamped()
    target.header.frame_id = 'world'
    target.pose.position.x = 0.300
    target.pose.position.y = 0.200
    target.pose.position.z = 0.290
    target.pose.orientation.x = -1.00
    target.pose.orientation.y =  0.00
    target.pose.orientation.z =  0.00
    target.pose.orientation.w =  0.00

    goal = MoveToPoseGoal()
    goal.pose_stamped=target
    pose_cli.send_goal(goal)
    pose_cli.wait_for_result()