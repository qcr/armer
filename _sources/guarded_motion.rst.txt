Guarded Motion
====================================

The Armer interface contains a ROS action server which listens for requests to set Cartesian velocities in a specified frame and respond reactively to force on the end effector.

The user can use a ROS action client to set the Cartesian velocities of the end effector.


Via Python
-----------------

Requests to the ROS action server can be made by creating a ROS action client in Python. This client is used to send requests to the action server.

The following code block shows a Python3 script of setting the linear velocty in the z direction to -0.05 m/s. The manipulator will continue in this direction for 5 seconds or until a force greater than 5nm applied to the end effector is detected.

.. code-block:: python

    import rospy
    import actionlib
    from armer_msgs.msg import GuardedVelocityAction, GuardedVelocityGoal

    rospy.init_node('armer_example')

    vel_cli = actionlib.SimpleActionClient('/arm/cartesian/guarded_velocity', GuardedVelocityAction)
    vel_cli.wait_for_server()

    goal = GuardedVelocityGoal()
    goal.twist_stamped.twist.linear.z = -0.05

    # Trigger on duration expiring or force limit violation
    goal.guards.enabled = goal.guards.GUARD_DURATION | goal.guards.GUARD_EFFORT 
    # motion should not last longer than 5 seconds
    goal.guards.duration = 5       
    # force measured at end-effector should not exceed 5nm
    goal.guards.effort.force.z = 5 

    vel_cli.send_goal(goal)
    vel_cli.wait_for_result()