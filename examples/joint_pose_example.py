from math import pi
from armer_msgs.msg import MoveToJointPoseAction, MoveToJointPoseGoal 
import actionlib
import rospy

rospy.init_node('armer_example', disable_signals=True)
servo_cli = actionlib.SimpleActionClient('/arm/joint/pose', MoveToJointPoseAction)
servo_cli.wait_for_server()

desired_joints=[0, -pi/2, pi/4, 0, 0, 0]

print('Moving joints to {}'.format(str(desired_joints)))

# Create goal from target pose
goal = MoveToJointPoseGoal()
goal.joints=desired_joints


# Send goal and wait for it to finish
servo_cli.send_goal(goal)
servo_cli.wait_for_result()
servo_cli.get_result()