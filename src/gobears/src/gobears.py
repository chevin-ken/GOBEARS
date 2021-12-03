#!/usr/bin/env python
import rospy
import sys
import cv2

from baxter import * 


TROWEL_AR_TAG_ID = 2
HOLE_AR_TAG_ID = 3
PLANT_AR_TAG_ID = 4
WATER_AR_TAG_ID = 5

class GoBears():
	def __init__(self):
		self.baxter = Baxter()
		#Set right camera to desired initial pose
		right_camera_pose = None
		orientation_constraints = []
		self.baxter.execute(right_camera_pose, orientation_constraints, "right")

	def test(self):
		ar_pose = self.baxter.get_ar_pose(0)
		orientation_constraints = []
		self.baxter.execute(ar_pose, orientation_constraints)	

	def dig(self):
		#Detect trowel
		trowel_ar_pose = self.baxter.get_ar_pose(TROWEL_AR_TAG_ID)

		#Compute desired relative final pose with respect to ar tag pose
		trowel_pose = trowel_ar_pose

		#Move to pose with orientation constraints
		orientation_constraints = []
		self.baxter.execute(trowel_pose, orientation_constraints, "left")

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
			self.baxter.execute(trowel_pose, orientation_constraints, "left")

			#Execute scooping motion
			self.baxter.scoop()

			#Determine completion somehow
			complete = True

	def plant(self):
		#Detect location of place to put trowel down
		trowel_ar_pose = self.baxter.get_ar_pose(TROWEL_AR_TAG_ID)

		#Compute desired relative final pose with respect to ar tag pose
		trowel_pose = trowel_ar_pose

		#Move to pose with orientation constraints
		orientation_constraints = []
		self.baxter.execute(trowel_pose, orientation_constraints, "left")

		#Release gripper to put down trowel
		self.baxter.release_gripper()

		#Detect location of plant 
		plant_ar_pose = self.baxter.get_ar_pose(PLANT_AR_TAG_ID)

		#Compute desired relative final pose with respect to ar tag pose
		plant_pose = plant_ar_pose

		#Move to pose with orientation constraints
		orientation_constraints = []
		self.baxter.execute(plant_pose, orientation_constraints, "left")

		#Detect location of hole 
		hole_ar_pose = self.baxter.get_ar_pose(HOLE_AR_TAG_ID)

		#Compute desired relative final pose with respect to hole pose
		hole_pose = hole_ar_pose

		#Move to pose with orientation constraints
		orientation_constraints = []
		self.baxter.execute(hole_pose, orientation_constraints, "left")

		#Release gripper to release plant
		self.baxter.release_gripper()

	def fill(self):
		#Do we actually need fill?
		return

	def water(self):
		#Find location of watering can 
		water_ar_pose = self.baxter.get_ar_pose(WATER_AR_TAG_ID)

		#Compute desired relative final pose with respect to ar tag pose
		water_pose = water_ar_pose

		#Move to pose with orientation constraints
		orientation_constraints = []
		self.baxter.execute(water_pose, orientation_constraints, "left")

		#Close gripper to pick up water
		self.baxter.close_gripper()

		#Detect location of hole
		hole_ar_pose = self.baxter.get_ar_pose(HOLE_AR_TAG_ID)

		#Compute desired relative final pose with respect to hole pose
		hole_pose = hole_ar_pose

		#Move to pose with orientation constraints
		orientation_constraints = []
		self.baxter.execute(hole_pose, orientation_constraints, "left")

		#Execute pouring motion
		self.baxter.pour()

		#Put water back down in original position
		water_ar_pose = self.baxter.get_ar_pose(WATER_AR_TAG_ID)

		#Compute desired relative final pose with respect to ar tag pose
		water_pose = water_ar_pose

		#Move to pose with orientation constraints
		orientation_constraints = []
		self.baxter.execute(water_pose, orientation_constraints, "left")

		#Release gripper to put down watering can
		self.baxter.release_gripper()

		#Return to a neutral position
		self.baxter.reset()

if __name__ == '__main__':
	gobears = GoBears()
	print("GoBears")
	gobears.test()
	print("Test ended")
	gobears.dig()
	print("Finished digging, now planting")
	gobears.plant()
	print("Finished planting now filling")
	gobears.fill()
	print("Finished filling now watering")
	gobears.water()
	print("All done!")