import rospy
import actionlib
from armer_msgs.msg import GuardedVelocityAction, GuardedVelocityGoal

rospy.init_node('armer_example')

vel_cli = actionlib.SimpleActionClient('/arm/cartesian/guarded_velocity', GuardedVelocityAction)
vel_cli.wait_for_server()

goal = GuardedVelocityGoal()
goal.twist_stamped.twist.linear.z = -0.05

# Trigger on duration expiring or force limit violation
goal.guards.enabled = goal.guards.GUARD_DURATION | goal.guards.GUARD_EFFORT 
# motion should not last longer than 5 seconds
goal.guards.duration = 5       
# force measured at end-effector should not exceed 5nm
goal.guards.effort.force.z = 5 

vel_cli.send_goal(goal)
vel_cli.wait_for_result()