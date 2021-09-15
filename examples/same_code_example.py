import rospy
from geometry_msgs.msg import TwistStamped
import timeit

# initialise ros node
rospy.init_node('move_to_points_example', disable_signals=True)
vel_pub = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

# Cartesian velocity example
DESIRED_TIME=2
print('Moving {} for {} seconds'.format('up', DESIRED_TIME))
ts = TwistStamped()
ts.twist.linear.z = 0.1

start_time = timeit.default_timer()

while timeit.default_timer() - start_time < DESIRED_TIME:
    vel_pub.publish(ts)

vel_pub.publish(TwistStamped())

print('Moving {} for {} seconds'.format('down', DESIRED_TIME))
ts = TwistStamped()
ts.twist.linear.z = -0.1

start_time = timeit.default_timer()

while timeit.default_timer() - start_time < DESIRED_TIME:
    vel_pub.publish(ts)

print('Moving {} for {} seconds'.format('forward', DESIRED_TIME))
ts = TwistStamped()
ts.twist.linear.x = 0.1

start_time = timeit.default_timer()

while timeit.default_timer() - start_time < DESIRED_TIME:
    vel_pub.publish(ts)

vel_pub.publish(TwistStamped())

print('Moving {} for {} seconds'.format('backwards', DESIRED_TIME))
ts = TwistStamped()
ts.twist.linear.x = -0.1

start_time = timeit.default_timer()

while timeit.default_timer() - start_time < DESIRED_TIME:
    vel_pub.publish(ts)

vel_pub.publish(TwistStamped())