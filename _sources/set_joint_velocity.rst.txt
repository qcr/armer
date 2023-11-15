Setting Joint Velocites
====================================

The Armer interface listens to ROS messages published on the ``/arm/joint/velocity`` topic to control joint velocity.

The user can publish to this topic to set the joint velocites of any or all of the joints on a manipulator.

Via Python
-----------------

ROS topics can be published to from a Python script using Rospy, the ROS Python client.

The following code example shows the setting of joint velocities on a robot with 6 joints. Joints 1, 3, 4, 5, and 6 are set to 0 rad/s while the second joint from the base  is set to rotate at 0.1 rad/s in a clockwise direction.. The number of elements in the vel_msg.joints array should match with the joints on the target robot.

.. warning::

    The joint will continue rotating till the script is killed (``ctrl+c``).
 
.. code-block:: python

    import rospy
    from armer_msgs.msg import JointVelocity
    
    # Initialise node
    rospy.init_node('armer_example', disable_signals=True)
    
    # Setup publisher
    vel_pub = rospy.Publisher('/arm/joint/velocity', JointVelocity, queue_size=1)
    
    # Create message type and populate
    vel_msg = JointVelocity()
    vel_msg.joints = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Define rate (100 Hz recommended)
    rate = rospy.Rate(100) #hz
    while not rospy.is_shutdown():
        # Publish velocity
        vel_pub.publish(vel_msg)
        # Tick the node
        rate.sleep()


The length of the vel_msg.joints array will be dependent on the number of joints in the target manipulator. This example shows a 7 jointed arm.

Testing from Command Line
---------------------------

Publishing to the joint velocity topic can be done from the command line. 

The ROS command to publish to a topic syntax begins with ``rostopic pub`` followed by the topic to publish to, the message type and finally the message itself to publish. This can be greatly simplfied on the command line by using tab to autocomplete.

.. note::

    Unfortunately the autocomplete this message type is broken and should be manually corrected from ``joints:- 0"`` to ``"joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"`` where the number of elements in the joints array corresponds to the number of joints on the manipulator.

The following code snippet displays an example of setting the velocty of the second joint of the manipulator to rotate at 0.1 rad/s.

.. warning::
    The ``-r`` argument tells the node to publish continuously at 100 Hz. This example will continuously request the second joint of the manipulator to rotate at 0.1 rad/s so the robot will move until killed (``ctrl+c``).

.. code-block:: bash

    rostopic pub -r 100 /arm/joint/velocity armer_msgs/JointVelocity "joints: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]" 

