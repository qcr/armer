Setting Cartesian Velocites
====================================

The Armer API listens to ROS messages published on the ``/arm/cartesian/velocity`` topic to interface with cartesian velocity in a specified frame.

The user can publish to this topic to set the cartesian velocites of the end effector.

Via Command Line
-----------------

Publishing to the velocity topic can be done from the command line. 

The ROS command to publish to a topic syntax begins with ``rostopic pub`` followed by the topic to publish to, the message type and finally the message itself to publish. This can be greatly simplfied on the command line by using tab to autocomplete.

The following code block shows and example of setting the linear velocty in the x direction to 0.1 m/s.

.. warning::
    The ``-r`` argument tells the node to publish continuously at 100 Hz. This example will continuously request a linear velocity in the x direction of 0.1 m/s so the robot will move until killed (``ctrl+c``).

.. code-block:: bash

    $ rostopic pub -r 100 /arm/cartesian/velocity geometry_msgs/TwistStamped "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
    twist:
    linear:
        x: 0.1
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.0" 

Via Python
-----------------

ROS topics can also be published to from a Python script using Rospy, the ROS Python client.

The following code block shows the same command demonstrated in the command line example implemented as a Python3 script. As with the command line example, the node continues to publish the message requesting a velocity of 0.1 m/s in the x direction so the manipulator will continue in the direction until the script is killed.

.. code-block:: python
    :linenos:

    import rospy
    from geometry_msgs.msg import TwistStamped

    rospy.init_node('armer_example', disable_signals=True)
    vel_pub = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)
    ts = TwistStamped()
    ts.twist.linear.x = 0.1

    while True:
        vel_pub.publish(ts)
