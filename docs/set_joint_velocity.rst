Setting Joint Velocites
====================================

The Armer API listens to ROS messages published on the ``/arm/joint/velocity`` topic to interface with joint velocity.

The user can publish to this topic to set the joint velocites of any or all of the joints on a manipulator.

Via Command Line
-----------------

Publishing to the joint velocity topic can be done from the command line. 

The ROS command to publish to a topic syntax begins with ``rostopic pub`` followed by the topic to publish to, the message type and finally the message itself to publish. This can be greatly simplfied on the command line by using tab to autocomplete.

.. note::
    Unfortunately the autocomplete this message type is broken and should be manually corrected from ``joints:- 0"`` to ``"joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"`` where the number of entries in the joints array corresponds to the number of joints on the manipulator.

The following code snippet displays an example of setting the velocty of the second joint of the manipulator to rotate at 0.1 rad/s.

.. warning::
    The ``-r`` argument tells the node to publish continuously at 100 Hz. This example will continuously request the second joint of the manipulator to rotate at 0.1 rad/s so the robot will move until killed (``ctrl+c``).

.. code-block:: bash

    $ rostopic pub -r 100 /arm/joint/velocity armer_msgs/JointVelocity "joints: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]" 

Via Python
-----------------

ROS topics can also be published to from a Python script using Rospy, the ROS Python client.

The following code block shows the same command demonstrated in the command line example implemented as a Python3 script. As with the command line example, the node continues to publish the message requesting the velocty of the second joint of the manipulator to rotate at 0.1 rad/s so the manipulator will continue in the direction until the script is killed.

.. code-block:: python
    :linenos:

    import rospy
    from armer_msgs.msg import JointVelocity

    rospy.init_node('armer_example', disable_signals=True)
    vel_pub = rospy.Publisher('/arm/joint/velocity', JointVelocity, queue_size=1)
    vel_msg = JointVelocity()
    vel_msg.joints = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]

    while True:
        vel_pub.publish(vel_msg)