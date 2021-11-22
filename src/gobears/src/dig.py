import rospy
import sys

done = False
success = False

#Returned message is combination of done and success
def callback():
	return

def gobears_dig():
	#Initialize vision client

	#Initialize control client

	#Initialize dig server

	#1: Pick up trowel
	#Call vision to find trowel, compute final orientation, call control to move to desired pose, close gripper

	#2: Scoop Loop
		#2.1: Find position of bean chunks, calculate desired pose, move to pose
		#2.2 Execute predefined joint motion for scooping
		#2.3 Find location of bin to put in loose bean chunks, calculate desired pose, move to pose
		#2.4 Execute predefined joint motion for dropping into bin
		#2.5 Call vision node to check size of hole, if vision node determines done, then terminate


def main():
	rospy.init_node('dig')
	gobears_dig():