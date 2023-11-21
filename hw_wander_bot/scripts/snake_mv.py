#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
import math
import time

class SnakeMv:
	LINEAR_SPEED = 0.1
	ANGULAR_SPEED = 0.1
	MIN_SCAN_ANGLE = math.pi + (-20 / 180 * math.pi)
	MAX_SCAN_ANGLE = math.pi + (20 / 180 * math.pi)
	ORIENT = 90
	MIN_DIST_FROM_OBSTACLE = 0.4
	DIST_SHORT_SIDE = 0.5


	def __init__(self):
		self.isObstacle = True
		self.isRotation = False
		self.isAfterOddRotation = False
		self.flagPosition = False
		self.flagOrient = False
		self.numRotation = 0
		self.startPositionX = 0
		self.startPositionY = 0
		self.startOrient = 0

		self.cmdVelPub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1000)

		self.modelStatesSub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelStatesCallback, queue_size = 1)

		self.scanSub = rospy.Subscriber("/scan", LaserScan, self.scanCallback, queue_size = 1)
	

	def moveLinear(self):
		msg = Twist()

		msg.linear.x = (-1) * self.LINEAR_SPEED
		msg.linear.y = 0
		msg.linear.z = 0
		msg.angular.x = 0
		msg.angular.y = 0
		msg.angular.z = 0
		self.cmdVelPub.publish(msg)


	def moveAngular(self, direction):
		msg = Twist()

		msg.linear.x = 0
		msg.linear.y = 0
		msg.linear.z = 0
		msg.angular.x = 0
		msg.angular.y = 0
		if (direction):
			msg.angular.z = self.ANGULAR_SPEED
		else:
			msg.angular.z = (-1) * self.ANGULAR_SPEED
		self.cmdVelPub.publish(msg)


	def stop(self):
		print("Stop!")
		msg = Twist()

		msg.linear.x = 0
		msg.linear.y = 0
		msg.linear.z = 0
		msg.angular.x = 0
		msg.angular.y = 0
		msg.angular.z = 0
		self.cmdVelPub.publish(msg)
		time.sleep(2)


	def initStartPositionXY(self, modelStates):
		if (self.flagPosition):
			self.startPositionX = modelStates.pose[2].position.x
			self.startPositionY = modelStates.pose[2].position.y
			self.flagPosition = False


	def initStartOrient(self, modelStates):
		if (self.flagOrient):
			self.startOrient = self.getCurOrient(modelStates)
			self.flagOrient = False


	def getCurOrient(self, modelStates):
		z = modelStates.pose[2].orientation.z
		w = modelStates.pose[2].orientation.w
		curOrient = 0

		if z <= 0 and w >= 0 or z > 0 and w < 0:
			curOrient = abs(2 * math.asin(z)) * 180 / math.pi
			if z > 0 and w < 0:
				curOrient = -curOrient
		else:
			curOrient = (math.pi + abs(2 * math.asin(w))) * 180 / math.pi
			if z > 0 and w >= 0:
				curOrient = -curOrient
		return curOrient


	def modelStatesCallback(self, modelStates):
		if (self.isRotation):
			self.initStartOrient(modelStates)
			curOrient = self.getCurOrient(modelStates)
			deltaOrient = abs(abs(curOrient) - abs(self.startOrient))

			if curOrient >= 0 and self.startOrient <= 0 or curOrient <= 0 and self.startOrient >= 0:
				deltaOrient = 360 - deltaOrient
			print("modelStatesCallback curOrient =", curOrient)
			print("modelStatesCallback deltaOrient =", deltaOrient)
			if deltaOrient >= self.ORIENT:
				print("modelStatesCallback isRotation")
				self.stop()
				if (self.numRotation % 2 == 0):
					self.isAfterOddRotation = True
				self.numRotation = (self.numRotation + 1) % 4
				self.isRotation = False
		if (self.isAfterOddRotation):
			self.initStartPositionXY(modelStates)
			currentPositionX = modelStates.pose[2].position.x
			currentPositionY = modelStates.pose[2].position.y
			currentDistShortSide = math.sqrt(pow(currentPositionX - self.startPositionX, 2) + pow(currentPositionY - self.startPositionY, 2))
			
			print("modelStatesCallback currentDistShortSide =", currentDistShortSide)
			if currentDistShortSide >= self.DIST_SHORT_SIDE:
				print("modelStatesCallback isAfterOddRotation")
				self.stop()
				self.flagOrient = True
				self.isRotation = True
				self.isAfterOddRotation = False


	def scanCallback(self, scan):
		isObstacleInFront = False
		minIndex = round((self.MIN_SCAN_ANGLE - scan.angle_min) / scan.angle_increment)
		maxIndex = round((self.MAX_SCAN_ANGLE - scan.angle_min) / scan.angle_increment)
		print("minIndex =", minIndex)
		print("maxIndex =", maxIndex)
		currIndex = minIndex

		while currIndex <= maxIndex:
			if scan.ranges[int(currIndex)] <= self.MIN_DIST_FROM_OBSTACLE:
				isObstacleInFront = True
				break
			currIndex += 1
		if isObstacleInFront:
			self.isObstacle = True
			if not self.isRotation:
				print("scanCallback isObstacle")
				self.stop()
				self.flagOrient = True
				self.isRotation = True
				self.isAfterOddRotation = False
				self.flagPosition = True
				print("scanCallback ranges")
				i = 0
				while i < 360:
					print("[%d] = %f", i, scan.ranges[i])
					i += 1
		else:
			self.isObstacle = False
	

	def startMoving(self):
		rospy.init_node("snake_mv", anonymous = False)
		rate = rospy.Rate(5000)
		while not rospy.is_shutdown():
			if not self.isObstacle and not self.isRotation:
				self.moveLinear()
			elif self.isRotation and not self.flagOrient:
				if self.numRotation < 2:
					self.moveAngular(False)
				else:
					self.moveAngular(True)
			rate.sleep()
			

if __name__ == '__main__':
	try:
		snakeMv = SnakeMv()
		snakeMv.startMoving()
	except rospy.ROSInterruptException:
		pass
