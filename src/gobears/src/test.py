#!/usr/bin/env python
import sys
import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize

rospy.init_node('foo', anonymous=False)
roscpp_initialize(sys.argv)

group = MoveGroupCommander('right_arm')
print group.get_current_pose()
print group.get_current_joint_values()