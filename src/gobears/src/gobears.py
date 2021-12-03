#!/usr/bin/env python
import rospy
import sys
import cv2

from baxter import * 


TROWEL_AR_TAG_ID = 2

class GoBears():
	def __init__(self):
		self.baxter = Baxter()

	def test(self):
		ar_pose = self.baxter.get_ar_pose(0)
		orientation_constraints = []
		self.baxter.execute(ar_pose, orientation_constraints)	

	def dig(self):
		#Step 1: detect trowel
		trowel_ar_pose = self.baxter.get_ar_pose(TROWEL_AR_TAG_ID)

		#Step 2: compute desired relative final pose with respect to ar tag pose
		trowel_pose = trowel_ar_pose

		#Step 3: Move to pose with orientation constraints
		orientation_constraints = []
		# baxter.execute(trowel_pose, orientation_constraints)

		#Step 4: Close gripper to pick up plant


	def plant(self):
		return

	def fill(self):
		return

	def water(self):
		return

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