import rospy
from geometry_msgs.msg import TwistStamped

rospy.init_node('armer_example', disable_signals=True)

vel_pub = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)

vel_msg = TwistStamped()
vel_msg.twist.linear.x = 0.05

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    vel_pub.publish(vel_msg)
    rate.sleep()
