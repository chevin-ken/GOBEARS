import rospy
import sys

def get_head_orientation():
	pose = None
	return pose

def get_hand_orientation():
	pose = None
	return pose

#Use color for object detection
def detect_object(detector, color):
	head_point, hand_point = detector.detect(color)
	head_pose = get_head_orientation()
	hand_pose = get_hand_orientation()
	return calculate_world_frame_coordinates(head_point, head_pose, hand_point, hand_pose)

def calculate_world_frame_coordinates(head_point, head_pose, hand_point, hand_pose):
	return

#Request has name of object and camera
def vision_callback(request):
	#Switch statements for object == something, then use certain detector logic
	return

def vision_server():
	return

def main():
	rospy.init_node('vision')
	vision_server()