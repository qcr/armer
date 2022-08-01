import rospy
import actionlib
from armer_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from geometry_msgs.msg import PoseStamped
import rospy

rospy.init_node('armer_example', disable_signals=True)
pose_cli = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
pose_cli.wait_for_server()

target = PoseStamped()

target.pose.position.x = 0.300
target.pose.position.y = 0.200
target.pose.position.z = 0.290
target.pose.orientation.x = -1.00
target.pose.orientation.y =  0.00
target.pose.orientation.z =  0.00
target.pose.orientation.w =  0.00

goal = MoveToPoseGoal()
goal.pose_stamped=target
pose_cli.send_goal(goal)
pose_cli.wait_for_result()