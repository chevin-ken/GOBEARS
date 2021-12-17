#!/usr/bin/env python
import rospy
import sys
import cv2
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import baxter
from baxter import * 
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from scipy.spatial.transform import Rotation as R
from moveit_msgs.msg import Constraints, OrientationConstraint
import copy

TROWEL_AR_TAG_ID = 2
HOLE_AR_TAG_ID = 1
HOLE_2_AR_TAG_ID = 4
PLANT_AR_TAG_ID = 3
WATER_AR_TAG_ID = 0


# grip position

# Translation: [0.840, 0.374, -0.187]
# Rotation: in Quaternion [0.016, 0.746, -0.002, 0.666]

# ar marker

# Translation: [0.751, 0.408, -0.236]
# Rotation: in Quaternion [0.026, 0.044, -0.725, 0.687]

# translation: [0.9, -0.35, 0.5]
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
		raw_input("Press <Enter> to move the left arm to trowel p0.016, 0.746, -0.002, 0.666ose: ")
		
		#Try to enforce the success of the motion after having a plan
		successful = False
		while not successful:
			try:
				self.baxter.execute(trowel_plan, "left")
				successfulget_plant_rel = True
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


	def get_plant_hover_pose(self, plant_ar_pose):
		# transform = Transform()
		# transform.rotation = plant_ar_pose.orientation
		# transform.translation = plant_ar_pose.position
		# g_ba = make_homog_from_transform(transform)
		# plant_pose = get_pose_from_homog(g_ba, baxter.G_AL_PLANT)

		# plant_pose = Pose()
		# plant_pose.position.x = plant_ar_pose.position.x + 0.035
		# plant_pose.position.y = plant_ar_pose.position.y
		# plant_pose.position.z = plant_ar_pose.position.z + 0.2
		# plant_pose.orientation.x = 0
		# plant_pose.orientation.y = 1
		# plant_pose.orientation.z = 0
		# plant_pose.orientation.w = 0
		tag_pose = plant_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		plant_pose = get_pose_from_homog(g_ba, baxter.G_AL_PLANT_HOVER)


		plant_pose_stamped = PoseStamped()
		plant_pose_stamped.pose = plant_pose
		plant_pose_stamped.header.frame_id = "base"

		return plant_pose_stamped

	def get_plant_pickup_pose(self, plant_ar_pose):
		# plant_pose = Pose()
		# plant_pose.position.x = plant_ar_pose.position.x + 0.035
		# plant_pose.position.y = plant_ar_pose.position.y
		# plant_pose.position.z = plant_ar_pose.position.z + 0.03
		# plant_pose.orientation.x = 0
		# plant_pose.orientation.y = 1
		# plant_pose.orientation.z = 0
		# plant_pose.orientation.w = 0
		# plant_pose_stamped = PoseStamped()
		# plant_pose_stamped.pose = plant_pose
		# plant_pose_stamped.header.frame_id = "base"

		# return plant_pose_stamped


		tag_pose = plant_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		plant_pose = get_pose_from_homog(g_ba, baxter.G_AL_PLANT_PICK)


		plant_pose_stamped = PoseStamped()
		plant_pose_stamped.pose = plant_pose
		plant_pose_stamped.header.frame_id = "base"

		return plant_pose_stamped

	def get_hole_hover_pose(self, hole_ar_pose):
		# hole_pose = Pose()
		# hole_pose.position.x = hole_ar_pose.position.x + 0.04
		# hole_pose.position.y = hole_ar_pose.position.y + 0.12
		# hole_pose.position.z = hole_ar_pose.position.z + 0.25
		# hole_pose.orientation.x = 0
		# hole_pose.orientation.y = 1
		# hole_pose.orientation.z = 0
		# hole_pose.orientation.w = 0
		# hole_pose_stamped = PoseStamped()
		# hole_pose_stamped.pose = hole_pose
		# hole_pose_stamped.header.frame_id = "base"
		# return hole_pose_stamped

		tag_pose = hole_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		plant_pose = get_pose_from_homog(g_ba, baxter.G_AL_PLANT_HOVER_2)


		plant_pose_stamped = PoseStamped()
		plant_pose_stamped.pose = plant_pose
		plant_pose_stamped.header.frame_id = "base"

		return plant_pose_stamped

	def get_hole_release_pose(self, hole_ar_pose):
		# hole_pose = Pose()
		# hole_pose.position.x = hole_ar_pose.position.x + 0.04
		# hole_pose.position.y = hole_ar_pose.position.y + 0.12
		# hole_pose.position.z = hole_ar_pose.position.z + 0.10
		# hole_pose.orientation.x = 0
		# hole_pose.orientation.y = 1
		# hole_pose.orientation.z = 0
		# hole_pose.orientation.w = 0

		# hole_pose_stamped = PoseStamped()
		# hole_pose_stamped.pose = hole_pose
		# hole_pose_stamped.header.frame_id = "base"
		# return hole_pose_stamped

		tag_pose = hole_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		plant_pose = get_pose_from_homog(g_ba, baxter.G_AL_PLANT_PLACE)


		plant_pose_stamped = PoseStamped()
		plant_pose_stamped.pose = plant_pose
		plant_pose_stamped.header.frame_id = "base"

		return plant_pose_stamped

	def get_dig_1_pose(self, hole_ar_pose):
		tag_pose = hole_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		dig_hover_pose = get_pose_from_homog(g_ba, baxter.G_AL_DIG_1)


		dig_hover_stamped = PoseStamped()
		dig_hover_stamped.pose = dig_hover_pose
		dig_hover_stamped.header.frame_id = "base"

		return dig_hover_stamped

	def get_dig_2_pose(self, hole_ar_pose):
		tag_pose = hole_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		dig_hover_pose = get_pose_from_homog(g_ba, baxter.G_AL_DIG_2)


		dig_hover_stamped = PoseStamped()
		dig_hover_stamped.pose = dig_hover_pose
		dig_hover_stamped.header.frame_id = "base"

		return dig_hover_stamped

	def get_dig_3_pose(self, hole_ar_pose):
		tag_pose = hole_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		dig_hover_pose = get_pose_from_homog(g_ba, baxter.G_AL_DIG_3)


		dig_hover_stamped = PoseStamped()
		dig_hover_stamped.pose = dig_hover_pose
		dig_hover_stamped.header.frame_id = "base"

		return dig_hover_stamped

	def get_water_hover_pose(self, water_ar_pose):
		# water_pose = Pose()
		# water_pose.position.x = water_ar_pose.position.x + 0.8
		# water_pose.position.y = water_ar_pose.position.y - 0.35
		# water_pose.position.z = water_ar_pose.position.z + 0.5
		# water_pose.orientation.x = 0
		# water_pose.orientation.y = 1
		# water_pose.orientation.z = 0
		# water_pose.orientation.w = 0
	
		


		tag_pose = water_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		pose = get_pose_from_homog(g_ba, baxter.G_AL_WATER_HOVER)


		water_pose_stamped = PoseStamped()
		water_pose_stamped.pose = pose
		water_pose_stamped.header.frame_id = "base"
		return water_pose_stamped

	def get_water_bin_hover_pose(self, bin_ar_pose):
		# water_pose = Pose()
		# water_pose.position.x = water_ar_pose.position.x + 0.8
		# water_pose.position.y = water_ar_pose.position.y - 0.35
		# water_pose.position.z = water_ar_pose.position.z + 0.5
		# water_pose.orientation.x = 0
		# water_pose.orientation.y = 1
		# water_pose.orientation.z = 0
		# water_pose.orientation.w = 0
	
		


		tag_pose = bin_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		pose = get_pose_from_homog(g_ba, baxter.G_AL_WATER_BIN_HOVER)


		water_pose_stamped = PoseStamped()
		water_pose_stamped.pose = pose
		water_pose_stamped.header.frame_id = "base"
		return water_pose_stamped

	def get_water_bin_water_pose(self, bin_ar_pose):
		# water_pose = Pose()
		# water_pose.position.x = water_ar_pose.position.x + 0.8
		# water_pose.position.y = water_ar_pose.position.y - 0.35
		# water_pose.position.z = water_ar_pose.position.z + 0.5
		# water_pose.orientation.x = 0
		# water_pose.orientation.y = 1
		# water_pose.orientation.z = 0
		# water_pose.orientation.w = 0
	
		


		tag_pose = bin_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		pose = get_pose_from_homog(g_ba, baxter.G_AL_WATER_BIN_WATER)


		water_pose_stamped = PoseStamped()
		water_pose_stamped.pose = pose
		water_pose_stamped.header.frame_id = "base"
		return water_pose_stamped

	def get_water_place_pose(self, water_ar_pose):
		# water_pose = Pose()
		# water_pose.position.x = water_ar_pose.position.x + 0.9
		# water_pose.position.y = water_ar_pose.position.y - 0.35
		# water_pose.position.z = water_ar_pose.position.z + 0.5
		# water_pose.orientation.x = 0
		# water_pose.orientation.y = 1
		# water_pose.orientation.z = 0
		# water_pose.orientation.w = 0
	
		# water_pose_stamped = PoseStamped()
		# water_pose_stamped.pose = water_pose
		# water_pose_stamped.header.frame_id = "base"
		# return water_pose_stamped

		tag_pose = water_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		pose = get_pose_from_homog(g_ba, baxter.G_AL_WATER_PICK)

		water_pose_stamped = PoseStamped()
		water_pose_stamped.pose = pose
		water_pose_stamped.header.frame_id = "base"
		return water_pose_stamped


	def get_hole_watering_pose(self, hole_ar_pose):
		watering_pose = Pose()
		watering_pose.position.x = hole_ar_pose.position.x + 0.0
		watering_pose.position.y = hole_ar_pose.position.y + 0.0
		watering_pose.position.z = hole_ar_pose.position.z + 0.0
		watering_pose.orientation.x = 0
		watering_pose.orientation.y = 1
		watering_pose.orientation.z = 0
		watering_pose.orientation.w = 0
	
		watering_pose_stamped = PoseStamped()
		watering_pose_stamped.pose = watering_pose
		watering_pose_stamped.header.frame_id = "base"
		return watering_pose_stamped

	def get_trowel_hover_pose(self, trowel_ar_pose):
		tag_pose = trowel_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		pose = get_pose_from_homog(g_ba, baxter.G_AL_SETUP)

		water_pose_stamped = PoseStamped()
		water_pose_stamped.pose = pose
		water_pose_stamped.header.frame_id = "base"
		return water_pose_stamped

	def get_trowel_pick_pose(self, trowel_ar_pose):
		tag_pose = trowel_ar_pose
		transform = Transform()
		transform.rotation = tag_pose.orientation
		transform.translation = tag_pose.position
		g_ba = make_homog_from_transform(transform)
		pose = get_pose_from_homog(g_ba, baxter.G_AL_FINAL)

		water_pose_stamped = PoseStamped()
		water_pose_stamped.pose = pose
		water_pose_stamped.header.frame_id = "base"
		return water_pose_stamped


	def dig(self):
		#Detect trowel
		# self.baxter.move_to_pose(baxter.RESET_POSE, [], "left")

		while self.baxter.get_ar_pose(TROWEL_AR_TAG_ID) == 0 or self.baxter.get_ar_pose(HOLE_AR_TAG_ID) == 0:
			self.baxter.rescan()
		trowel_ar_pose = self.baxter.get_ar_pose(TROWEL_AR_TAG_ID)
		self.baxter.change_velocity(0.5)

		#Compute desired relative final pose with respect to ar tag pose
		orientation_constraints = []
		trowel_hover_pose = self.get_trowel_hover_pose(trowel_ar_pose)
		self.baxter.move_to_pose(trowel_hover_pose, orientation_constraints, "left")

		# orientation_constraints = []
		# self.baxter.move_to_pose(dig_1_pose, orientation_constraints, "left")

		orientation_constraints = []
		trowel_pick_pose = self.get_trowel_pick_pose(trowel_ar_pose)
		self.baxter.move_to_pose(trowel_pick_pose, orientation_constraints, "left")

		self.baxter.close_gripper()
		self.baxter.change_velocity(0.75)
		trowel_hover_pose.pose.position.z += 0.15
		self.baxter.move_to_pose(trowel_hover_pose, orientation_constraints, "left")

		# self.baxter.move_to_pose(baxter.RESET_POSE, [], "left")
		bin_ar_pose = self.baxter.get_ar_pose(HOLE_AR_TAG_ID)
		dig_1_pose = self.get_dig_1_pose(bin_ar_pose)
		dig_1_pose.pose.orientation = Quaternion()
		dig_1_pose.pose.orientation.y = 1

		#Move to pose with orientation constraints
		orientation_constraints = []

		self.baxter.move_to_pose(dig_1_pose, orientation_constraints, "left")

		#dig 1

		dig_2_pose = self.get_dig_2_pose(bin_ar_pose)
		orientation_constraints = []
		self.baxter.move_to_pose(dig_2_pose, orientation_constraints, "left")

		#dig 2

		dig_3_pose = self.get_dig_3_pose(bin_ar_pose)
		orientation_constraints = []
		self.baxter.move_to_pose(dig_3_pose, orientation_constraints, "left")

		self.baxter.move_to_pose(baxter.RESET_POSE, [], "left")
		# dig_1_pose = self.get_dig_1_pose(bin_ar_pose)
		# orientation_constraints = []
		# self.baxter.move_to_pose(dig_1_pose, orientation_constraints, "left")

		# r = get_r_from_quaternion(hole_tilt_pose.pose.orientation)
		# roll, pitch, yaw = r.as_euler('xyz', degrees = True)
		# pitch -= 45
		# r = R.from_euler('xyz', [roll, pitch, yaw], degrees = True)
		# hole_tilt_pose.pose.orientation = get_quaternion_from_r(r)
		# hole_tilt_pose.pose.position.x -= .05

		# tilt_constraint = OrientationConstraint()
		# orientation_constraints = []
		# self.baxter.move_to_pose(dig_1_pose, orientation_constraints, "left")
		# tilt_constraint.orientation = hole_tilt_pose.pose.orientation
		# tilt_constraint.absolute_x_axis_tolerance = .2
		# tilt_constraint.absolute_y_axis_tolerance = 1.8
		# tilt_constraint.absolute_z_axis_tolerance = .2
		# orientation_constraints = [tilt_constraint]
		# self.baxter.move_to_pose(hole_tilt_pose, orientation_constraints, "left")

		# hole_tilt_dig_pose = copy.deepcopy(hole_tilt_pose)
		# hole_tilt_dig_pose.pose.position.z -= .03
		# self.baxter.move_to_pose(hole_tilt_dig_pose, orientation_constraints, "left")

		# hole_tilt_dig_pose.pose.position.x += 0.05
		# self.baxter.move_to_pose(hole_tilt_dig_pose, orientation_constraints, "left")
		# #Execute scooping motion
		# # self.baxter.scoop()

		#Move back to trowel hover pose
		orientation_constraints = []
		self.baxter.move_to_pose(trowel_hover_pose, orientation_constraints, "left")

		#Move to trowel pickup pose
		orientation_constraints = []
		self.baxter.move_to_pose(trowel_pick_pose, orientation_constraints, "left")

		#Release trowel
		raw_input("Press enter to open gripper")
		self.baxter.open_gripper()

	def plant(self):
		self.baxter.change_velocity(0.5)
		self.baxter.move_to_pose(baxter.RESET_POSE, [], "left")
		self.baxter.rescan()
		while self.baxter.get_ar_pose(PLANT_AR_TAG_ID) == 0 or self.baxter.get_ar_pose(HOLE_AR_TAG_ID) == 0:
			self.baxter.rescan()


		#Detect location of plant 
		plant_ar_pose = self.baxter.get_ar_pose(PLANT_AR_TAG_ID)

		#Compute desired relative final pose with respect to ar tag pose
		# plant_pose = self.get_plant_relative_pose(plant_ar_pose)
		plant_hover_pose = self.get_plant_hover_pose(plant_ar_pose)
		plant_hover_pose.pose.position.y -= .01
		orientation_constraints = []
		self.baxter.move_to_pose(plant_hover_pose, orientation_constraints, "left")

		#Move to pose with orientation constraints
		plant_pickup_pose = self.get_plant_pickup_pose(plant_ar_pose)
		plant_pickup_pose.pose.position.y -= .01
		self.baxter.move_to_pose(plant_pickup_pose, orientation_constraints, "left")

		rospy.sleep(1)
		# Close gripper
		self.baxter.close_gripper()

		# Move up 
		self.baxter.move_to_pose(plant_hover_pose, orientation_constraints, "left")


		hole_ar_pose = self.baxter.get_ar_pose(HOLE_AR_TAG_ID)
		print(hole_ar_pose)
		
		hole_hover_pose = self.get_hole_hover_pose(hole_ar_pose)
		orientation_constraints = []
		self.baxter.move_to_pose(hole_hover_pose, orientation_constraints, "left")
		

		#Move to pose with orientation constraints
		hole_release_pose = self.get_hole_release_pose(hole_ar_pose)
		self.baxter.move_to_pose(hole_release_pose, orientation_constraints, "left")


		rospy.sleep(1)
		# Open gripper
		self.baxter.open_gripper()
		self.baxter.move_to_pose(hole_hover_pose, orientation_constraints, "left")
		
	def water(self):
		#Find location of watering can 
		self.baxter.change_velocity(0.75)
		water_reset = copy.deepcopy(baxter.RESET_POSE)

		self.baxter.rescan()
		while self.baxter.get_ar_pose(WATER_AR_TAG_ID) == 0 or self.baxter.get_ar_pose(HOLE_AR_TAG_ID) == 0:
			self.baxter.rescan()

		water_reset.pose.position.x -= .15
		water_reset.pose.position.z -= .15
		self.baxter.move_to_pose(water_reset, [], "left")

		water_ar_pose = self.baxter.get_ar_pose(WATER_AR_TAG_ID)
		bin_ar_pose = self.baxter.get_ar_pose(HOLE_AR_TAG_ID)

		#Compute desired relative final pose with respect to ar tag pose
		water_hover_pose = self.get_water_hover_pose(water_ar_pose)

		#Move to pose with orientation constraints
		orientation_constraints = []
		self.baxter.move_to_pose(water_hover_pose, orientation_constraints, "left")


		water_pick_pose = self.get_water_place_pose(water_ar_pose)
		water_pick_pose.pose.position.z -= .03
		orientation_constraints = []
		self.baxter.move_to_pose(water_pick_pose, orientation_constraints, "left")


		tilt_constraint = OrientationConstraint()
		
		# allow 15 degrees of tilt in rpy when moving to hovering over bin
		tilt_constraint.orientation = water_pick_pose.pose.orientation
		tilt_constraint.absolute_x_axis_tolerance = .20
		tilt_constraint.absolute_y_axis_tolerance = .20
		tilt_constraint.absolute_z_axis_tolerance = 1.8
		orientation_constraints = [tilt_constraint]

		print("here")
		self.baxter.close_gripper()
		water_lift_pose = self.get_water_place_pose(water_ar_pose)
		water_lift_pose.pose.position.z += .1
		self.baxter.move_to_pose(water_lift_pose, orientation_constraints, "left")

		water_bin_hover_pose = self.get_water_bin_hover_pose(bin_ar_pose)
		water_bin_hover_pose.pose.position.z += 0.05
		self.baxter.move_to_pose(water_bin_hover_pose, orientation_constraints, "left")

		tilt_constraint.absolute_y_axis_tolerance = 1.8
		# tilt_constraint.absolute_z_axis_tolerance = .20

		orientation_constraints = [tilt_constraint]
		water_bin_water_pose = self.get_water_bin_water_pose(bin_ar_pose)
		water_bin_water_pose.pose.position.z += 0.05
		self.baxter.move_to_pose(water_bin_water_pose, orientation_constraints, "left")
		tilt_constraint.absolute_y_axis_tolerance = .2
		self.baxter.move_to_pose(water_lift_pose, orientation_constraints, "left")

		self.baxter.move_to_pose(water_pick_pose, orientation_constraints, "left")
		self.baxter.open_gripper()



if __name__ == '__main__':
	gobears = GoBears()
	print("GoBears")
	# gobears.baxter.test()

	# gobears.test()
	# gobears.test3()
	# print("Test ended")
	gobears.dig()
	# # print("Finished digging, now planting")
	gobears.plant()
	# print("Finished filling now watering")
	gobears.water()
	print("All done!")