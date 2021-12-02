#!/usr/bin/env python
import rospy
import sys
import cv2

from baxter import * 

class GoBears():
	def __init__(self):
		self.baxter = Baxter()

	def dig(self):
		image = self.baxter.get_image()
		print(type(image))
		cv2.imshow("", image)
		cv2.waitKey(0)
	def plant(self):
		return
	def fill(self):
		return
	def water(self):
		return

if __name__ == '__main__':
	print('did it reach main')
	gobears = GoBears()
	print("GoBears")
	gobears.dig()
	print("Finished digging, now planting")
	gobears.plant()
	print("Finished planting now filling")
	gobears.fill()
	print("Finished filling now watering")
	gobears.water()
	print("All done!")