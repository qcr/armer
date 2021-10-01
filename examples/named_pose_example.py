# from armer_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseGoal 
# from armer_msgs.srv import AddNamedPose
# import actionlib
# import rospy

# rospy.init_node('armer_example', disable_signals=True)

# add_pose_service = rospy.ServiceProxy('/arm/set_named_pose', AddNamedPose)

# named_pose="ur5_better_home"
# add_pose_service(named_pose, True)

# servo_cli = actionlib.SimpleActionClient('/arm/joint/named', MoveToNamedPoseAction)
# servo_cli.wait_for_server()

# print('Moving to named pose {}'.format(str(named_pose)))

# goal = MoveToNamedPoseGoal()
# goal.pose_name=named_pose

# # Send goal and wait for it to finish
# servo_cli.send_goal(goal)
# servo_cli.wait_for_result()
# servo_cli.get_result()

# Saving a pose example

from armer_msgs.srv import AddNamedPose
import rospy

rospy.init_node('armer_example', disable_signals=True)
add_pose_service = rospy.ServiceProxy('/arm/set_named_pose', AddNamedPose)

named_pose="my_pose"
add_pose_service(named_pose, True)

# Moving to a named pose example

# from armer_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseGoal
# import actionlib
# import rospy

# rospy.init_node('armer_example', disable_signals=True)

# named_pose="my_pose"

# servo_cli = actionlib.SimpleActionClient('/arm/joint/named', MoveToNamedPoseAction)
# servo_cli.wait_for_server()

# goal = MoveToNamedPoseGoal()
# goal.pose_name=named_pose

# # Send goal and wait for it to finish
# servo_cli.send_goal(goal)
# servo_cli.wait_for_result()
# servo_cli.get_result()

# Remove pose example
# from armer_msgs.srv import RemoveNamedPose
# import rospy

# rospy.init_node('armer_example', disable_signals=True)
# remove_pose_service = rospy.ServiceProxy('/arm/remove_named_pose', RemoveNamedPose)

# named_pose="my_pose"
# remove_pose_service(named_pose)

from armer_msgs.srv import GetNamedPoses
import rospy

rospy.init_node('armer_example', disable_signals=True)
get_poses_service = rospy.ServiceProxy('/arm/get_named_poses', GetNamedPoses)

get_poses_service()