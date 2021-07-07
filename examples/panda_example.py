#!/usr/bin/env python3
"""
Test script

.. codeauthor:: Gavin Suddreys
"""
import timeit

import rospy
import actionlib

from armer_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from armer_msgs.msg import ServoToPoseAction, ServoToPoseGoal
from geometry_msgs.msg import PoseStamped, TwistStamped

from std_srvs.srv import Empty

# initialise ros node
rospy.init_node('move_to_points_example', anonymous=True)


home_srv = rospy.ServiceProxy('/arm/home', Empty)

vel_pub = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

# Create a ros action client to communicate with the driver
pose_cli = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
pose_cli.wait_for_server()

servo_cli = actionlib.SimpleActionClient('/arm/cartesian/servo_pose', ServoToPoseAction)
servo_cli.wait_for_server()

# Create a target pose
target = PoseStamped()
target.header.frame_id = 'panda_link0'

# Populate with target position/orientation (READY POSE)
target.pose.position.x = 0.307
target.pose.position.y = 0.400
target.pose.position.z = 0.490

target.pose.orientation.x = -1.00
target.pose.orientation.y =  0.00
target.pose.orientation.z =  0.00
target.pose.orientation.w =  0.00

print('Moving to {}'.format(str(target.pose)))

# Create goal from target pose
goal = MoveToPoseGoal()
goal.pose_stamped=target

# Send goal and wait for it to finish
pose_cli.send_goal(goal)
pose_cli.wait_for_result()

# print('Moving {}'.format('Home'))
# home_srv()

# DESIRED_TIME=2
# print('Moving {} for {} seconds'.format('up', DESIRED_TIME))
# ts = TwistStamped()
# ts.twist.linear.z = 0.1

# start_time = timeit.default_timer()

# while timeit.default_timer() - start_time < DESIRED_TIME:
#     vel_pub.publish(ts)

# vel_pub.publish(TwistStamped())

# print('Moving {} for {} seconds'.format('down', DESIRED_TIME))
# ts = TwistStamped()
# ts.twist.linear.z = -0.1

# start_time = timeit.default_timer()

# while timeit.default_timer() - start_time < DESIRED_TIME:
#     vel_pub.publish(ts)

# vel_pub.publish(TwistStamped())


# Create a target pose
# target = PoseStamped()
# target.header.frame_id = 'panda_link0'

# # Populate with target position/orientation (READY POSE)
# target.pose.position.x = 0.307
# target.pose.position.y = 0.400
# target.pose.position.z = 0.490

# target.pose.orientation.x = -1.00
# target.pose.orientation.y =  0.00
# target.pose.orientation.z =  0.00
# target.pose.orientation.w =  0.00

# print('Servoing to {}'.format(str(target.pose)))

# # Create goal from target pose
# goal = ServoToPoseGoal()

# # Send goal and wait for it to finish
# servo_cli.send_goal(goal)
# servo_cli.wait_for_result()

# servo_cli.get_result()
