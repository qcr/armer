import rospy
from armer_msgs.msg import JointVelocity

rospy.init_node('armer_example', disable_signals=True)
vel_pub = rospy.Publisher('/arm/joint/velocity', JointVelocity, queue_size=1)
vel_msg = JointVelocity()
vel_msg.joints = [0.0, -0.1, 0.0, 0.0, 0.0, 0.0]

while True:
    vel_pub.publish(vel_msg)