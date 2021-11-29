import rospy
import sys

from baxter import * 

class GoBears():
	def __init__(self):
		self.baxter = Baxter()

	def dig(self):
		return
	def plant(self):
		return
	def fill(self):
		return
	def water(self):
		return

def __main__():
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