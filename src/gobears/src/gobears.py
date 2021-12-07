#!/usr/bin/env python
import rospy
import sys
import cv2
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import baxter
from baxter import * 
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.transform import Rotation as R
from moveit_msgs.msg import Constraints, OrientationConstraint

TROWEL_AR_TAG_ID = 0
HOLE_AR_TAG_ID = 1
HOLE_2_AR_TAG_ID = 2
PLANT_AR_TAG_ID = 3
WATER_AR_TAG_ID = 4

class GoBears():
	def __init__(self):
		self.baxter = Baxter()
		#Set right camera to desired initial pose
		right_camera_pose = None
		orientation_constraints = []
		# self.baxter.execute(right_camera_pose, orientation_constraints, "right")

	def test(self):
		ar_pose = self.baxter.get_ar_pose(0)
		print(type(ar_pose))
		
		ar_pose.position.z += 0.2
		ar_pose.orientation.x = 0.0
		ar_pose.orientation.y = 1
		ar_pose.orientation.z = 0.0
		ar_pose.orientation.w = 0.0

		orientation_constraints = []
		print(ar_pose)
		plan = self.baxter.plan(ar_pose, orientation_constraints, "left")
		print(plan)
		raw_input("Press <Enter> to move the left arm to goal pose 1: ")
		self.baxter.execute(plan, "left")

	def test2(self):	
		left_gripper_pose = gobears.baxter.lookup_transform('base', 'left_gripper')
		travel_pose = Pose()
		travel_pose.orientation = left_gripper_pose.transform.rotation
		travel_pose.position = left_gripper_pose.transform.translation
		initial_quaternions = [travel_pose.orientation.x, travel_pose.orientation.y, travel_pose.orientation.z, travel_pose.orientation.w]
		initial_euler = list(euler_from_quaternion(initial_quaternions))
		print(initial_euler)
		goal_euler = initial_euler
		goal_euler[1] += .5

		goal_quaternion = quaternion_from_euler(goal_euler[0], goal_euler[1], goal_euler[2])
		travel_pose.orientation.x = goal_quaternion[0]
		travel_pose.orientation.y = goal_quaternion[1]
		travel_pose.orientation.z = goal_quaternion[2]
		travel_pose.orientation.w = goal_quaternion[3]
		# goal_euler = initial_euler[1]
		# # print(type(left_gripper_pose))
		orientation_constraints = []
		plan = self.baxter.plan(travel_pose, orientation_constraints, "left")
		print(plan)
		raw_input("Press <Enter> to move the left arm to goal pose 1: ")
		self.baxter.execute(plan, "left")

	# tests dig motion without tilting the trowel up/down yet
	def test3(self):
		tag_pose = self.baxter.get_ar_pose(2)
		print(tag_pose)
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		trowel_pose = get_pose_from_homog(g_ba, baxter.G_AL_SETUP)

		tilt_constraint = OrientationConstraint()
		

		tilt_constraint.orientation = trowel_pose.orientation
		tilt_constraint.absolute_x_axis_tolerance = .05
		tilt_constraint.absolute_y_axis_tolerance = 1.8
		tilt_constraint.absolute_z_axis_tolerance = .05
		orientation_constraints = [tilt_constraint]
		trowel_plan = self.baxter.plan(trowel_pose, orientation_constraints, "left")
		raw_input("Press <Enter> to move the left arm to trowel pose: ")
		
		#Try to enforce the success of the motion after having a plan
		successful = False
		while not successful:
			try:
				self.baxter.execute(trowel_plan, "left")
				successful = True
			except:
				print("failed: retrying...")


		r = get_r_from_quaternion(trowel_pose.orientation)
		roll, pitch, yaw = r.as_euler('xyz', degrees = True)
		pitch += 45
		r = R.from_euler('xyz', [roll, pitch, yaw], degrees = True)
		trowel_pose.orientation = get_quaternion_from_r(r)



		orientation_constraints = []
		# trowel_pose = get_pose_from_homog(g_ba, baxter.G_AL_FINAL)
		trowel_plan = self.baxter.plan(trowel_pose, orientation_constraints, "left")
		raw_input("Press <Enter> to move the left arm to trowel pose: ")
		self.baxter.execute(trowel_plan, "left")

		forward_plan = self.baxter.move_forward(.1)
		raw_input("Press <Enter> to move the left arm to trowel pose: ")
		self.baxter.execute(forward_plan, "left")

		up_plan = self.baxter.move_up(.1)
		raw_input("Press <Enter> to move the left arm to trowel pose: ")
		self.baxter.execute(up_plan, "left")


	# tests getting in position to pick up trowel
	def test4(self):
		tag_pose = self.baxter.get_ar_pose(2)
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		trowel_pose = get_pose_from_homog(g_ba, baxter.G_AL_SETUP)

		orientation_constraints = []
		trowel_plan = self.baxter.plan(trowel_pose, orientation_constraints, "left")
		raw_input("Press <Enter> to move the left arm to trowel pose: ")
		self.baxter.execute(trowel_plan, "left")

		orientation_constraints = []
		trowel_pose = get_pose_from_homog(g_ba, baxter.G_AL_FINAL)
		trowel_plan = self.baxter.plan(trowel_pose, orientation_constraints, "left")
		raw_input("Press <Enter> to move the left arm to trowel pose: ")
		self.baxter.execute(trowel_plan, "left")


	def dig(self):
		#Detect trowel
		trowel_ar_pose = self.baxter.get_ar_pose(TROWEL_AR_TAG_ID)

		#Compute desired relative final pose with respect to ar tag pose
		trowel_pose = trowel_ar_pose

		#Move to pose with orientation constraints
		orientation_constraints = []
		trowel_plan = self.baxter.plan(trowel_pose, orientation_constraints, "left")
		raw_input("Press <Enter> to move the left arm to trowel pose: ")
		self.baxter.execute(trowel_plan, "left")

		#Close gripper to pick up trowel
		self.baxter.close_gripper()

		#Dig Loop
		complete = False
		while not complete:
			#Detect location of hole
			hole_ar_pose = self.baxter.get_ar_pose(HOLE_AR_TAG_ID)

			#Compute desired relative final pose with respect to hole pose
			hole_pose = hole_ar_pose

			#Move to pose with orientation constraints
			orientation_constraints = []

			hole_plan = self.baxter.plan(hole_pose, orientation_constraints, "left")
			raw_input("Press <Enter> to move the left arm to hole pose: ")
			self.baxter.execute(hole_plan, "left")

			#Execute scooping motion
			self.baxter.scoop()

			#Determine completion somehow
			complete = True




	# def plant(self):
	# 	#Detect location of place to put trowel down
	# 	trowel_ar_pose = self.baxter.get_ar_pose(TROWEL_AR_TAG_ID)

	# 	#Compute desired relative final pose with respect to ar tag pose
	# 	trowel_pose = trowel_ar_pose

	# 	#Move to pose with orientation constraints
	# 	orientation_constraints = []
	# 	self.baxter.execute(trowel_pose, orientation_constraints, "left")

	# 	#Release gripper to put down trowel
	# 	self.baxter.release_gripper()

	# 	#Detect location of plant 
	# 	plant_ar_pose = self.baxter.get_ar_pose(PLANT_AR_TAG_ID)

	# 	#Compute desired relative final pose with respect to ar tag pose
	# 	plant_pose = plant_ar_pose

	# 	#Move to pose with orientation constraints
	# 	orientation_constraints = []
	# 	self.baxter.execute(plant_pose, orientation_constraints, "left")

	# 	#Detect location of hole 
	# 	hole_ar_pose = self.baxter.get_ar_pose(HOLE_AR_TAG_ID)

	# 	#Compute desired relative final pose with respect to hole pose
	# 	hole_pose = hole_ar_pose

	# 	#Move to pose with orientation constraints
	# 	orientation_constraints = []
	# 	self.baxter.execute(hole_pose, orientation_constraints, "left")

	# 	#Release gripper to release plant
	# 	self.baxter.release_gripper()

	# def water(self):
	# 	#Find location of watering can 
	# 	water_ar_pose = self.baxter.get_ar_pose(WATER_AR_TAG_ID)

	# 	#Compute desired relative final pose with respect to ar tag pose
	# 	water_pose = water_ar_pose

	# 	#Move to pose with orientation constraints
	# 	orientation_constraints = []
	# 	self.baxter.execute(water_pose, orientation_constraints, "left")

	# 	#Close gripper to pick up water
	# 	self.baxter.close_gripper()

	# 	#Detect location of hole
	# 	hole_ar_pose = self.baxter.get_ar_pose(HOLE_AR_TAG_ID)

	# 	#Compute desired relative final pose with respect to hole pose
	# 	hole_pose = hole_ar_pose

	# 	#Move to pose with orientation constraints
	# 	orientation_constraints = []
	# 	self.baxter.execute(hole_pose, orientation_constraints, "left")

	# 	#Execute pouring motion
	# 	self.baxter.pour()

	# 	#Put water back down in original position
	# 	water_ar_pose = self.baxter.get_ar_pose(WATER_AR_TAG_ID)

	# 	#Compute desired relative final pose with respect to ar tag pose
	# 	water_pose = water_ar_pose

	# 	#Move to pose with orientation constraints
	# 	orientation_constraints = []
	# 	self.baxter.execute(water_pose, orientation_constraints, "left")

	# 	#Release gripper to put down watering can
	# 	self.baxter.release_gripper()

	# 	#Return to a neutral position
	# 	self.baxter.reset()

if __name__ == '__main__':
	gobears = GoBears()
	print("GoBears")
	# gobears.test()
	gobears.test3()
	print("Test ended")
	# gobears.dig()
	# print("Finished digging, now planting")
	# gobears.plant()
	# print("Finished filling now watering")
	# gobears.water()
	print("All done!")