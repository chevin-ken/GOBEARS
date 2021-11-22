import rospy
import actionlib
import sys

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveGroupFeedback, MoveGroupResult, JointConstraint, Constraints

def main():
	rospy.init_node('control')

	#Create subscriber to subscribe to node that receives position

	#Create publisher to publish success/failure of action and final configuration

