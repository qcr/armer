Setting Joint Positions
====================================

The Armer API contains a ROS action server which listens for requests to set of joint positions.

The user can use a ROS action client to set the joint positions of any or all of the joints on a manipulator.


Via Python
-----------------

Requests to the ROS action server can be made by creating a ROS action client in Python. This client is used to send requests to the action server.


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

Via Command Line
-----------------
While not entirely offical, requests to the action server can be made on the command line using tab auto-complete.

.. code-block:: bash

    $ rostopic pub /arm/cartesian/pose/goal armer_msgs/MoveToPoseActionGoal "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
    goal_id:
    stamp:
        secs: 0
        nsecs: 0
    id: ''
    goal:
    pose_stamped:
        header:
        seq: 0
        stamp:
            secs: 0
            nsecs: 0
        frame_id: ''
        pose:
        position:
            x: 0.0
            y: 0.0
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.0
            w: 0.0
    speed: 0.0" 

Publishing to the joint velocity topic can be done from the command line. 

The ROS command to publish to a topic syntax begins with ``rostopic pub`` followed by the topic to publish to, the message type and finally the message itself to publish. This can be greatly simplfied on the command line by using tab to autocomplete.

.. note::
    Unfortunately the autocomplete this message type is broken and should be manually corrected from ``joints:- 0"`` to ``"joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"`` where the number of entries in the joints array corresponds to the number of joints on the manipulator.

The following code snippet displays an example of setting the velocty of the second joint of the manipulator to rotate at 0.1 rad/s.

.. warning::
    The ``-r`` argument tells the node to publish continuously at 100 Hz. This example will continuously request the second joint of the manipulator to rotate at 0.1 rad/s so the robot will move until killed (``ctrl+c``).

.. code-block:: bash

    $ rostopic pub -r 100 /arm/joint/velocity armer_msgs/JointVelocity "joints: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]" 
