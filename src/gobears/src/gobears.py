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

class GoBears():
	def __init__(self):
		self.baxter = Baxter()

	def get_plant_hover_pose(self, plant_ar_pose):
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
		while self.baxter.get_ar_pose(TROWEL_AR_TAG_ID) == 0 or self.baxter.get_ar_pose(HOLE_AR_TAG_ID) == 0:
			self.baxter.rescan()
		trowel_ar_pose = self.baxter.get_ar_pose(TROWEL_AR_TAG_ID)
		self.baxter.change_velocity(0.5)

		#Compute desired relative final pose with respect to ar tag pose
		orientation_constraints = []
		trowel_hover_pose = self.get_trowel_hover_pose(trowel_ar_pose)
		self.baxter.move_to_pose(trowel_hover_pose, orientation_constraints, "left")

		orientation_constraints = []
		trowel_pick_pose = self.get_trowel_pick_pose(trowel_ar_pose)
		self.baxter.move_to_pose(trowel_pick_pose, orientation_constraints, "left")

		self.baxter.close_gripper()
		self.baxter.change_velocity(0.75)
		trowel_hover_pose.pose.position.z += 0.15
		self.baxter.move_to_pose(trowel_hover_pose, orientation_constraints, "left")

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
	gobears.dig()
	gobears.plant()
	gobears.water()
	print("All done!")