import rospy
from armer_msgs.srv import GetNamedPoses, AddNamedPose

rospy.init_node('armer_example', disable_signals=True)

get_poses_service = rospy.ServiceProxy('/arm/get_named_poses', GetNamedPoses)
add_pose_service = rospy.ServiceProxy('/arm/set_named_pose', AddNamedPose)

named_pose="my_pose"
add_pose_service(named_pose, True)

print(get_poses_service())