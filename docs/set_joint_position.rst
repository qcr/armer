Setting Joint Positions
====================================

The Armer API contains a ROS action server which listens for requests to set of joint positions.

The user can use a ROS action client to set the joint positions of any or all of the joints on a manipulator.


Via Python
-----------------

Requests to the ROS action server can be made by creating a ROS action client in Python. This client is used to send requests to the action server.

This example shows the setting of all the joints to 0 radians and the second joint to 0.1 radians.

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
