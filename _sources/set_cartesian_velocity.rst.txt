Setting Cartesian Velocites
====================================

The Armer interface listens to ROS messages published on the ``/arm/cartesian/velocity`` topic to move with Cartesian velocity in a specified frame.

Users can publish to this topic to set the Cartesian velocities of the end effector.


Via Python
-----------------

ROS topics can be published in a Python script using Rospy, the ROS Python client.

The following code block shows a Python3 script of setting the linear velocty in the x direction to 0.1 m/s. The node continues to publish the message requesting a velocity of 0.1 m/s in the x direction so the manipulator will continue in this direction until the script is killed.

.. warning::
    This example will continuously request a linear velocity in the x direction of 0.1 m/s so the robot will move until killed (``ctrl+c``).

.. code-block:: python

    import rospy
    from geometry_msgs.msg import TwistStamped

    # Initialise node
    rospy.init_node('armer_example', disable_signals=True)
    
    # Start publisher to topic
    vel_pub = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)
    
    # Create message type and populate
    vel_msg = TwistStamped()
    vel_msg.twist.linear.x = 0.1

    # Define rate (100 Hz recommended)
    rate = rospy.Rate(100) #hz
    while not rospy.is_shutdown():
        # Publish velocity
        vel_pub.publish(vel_msg)
        # Tick the node
        rate.sleep()
