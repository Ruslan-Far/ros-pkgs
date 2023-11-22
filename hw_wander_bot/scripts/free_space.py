#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
import math

class FreeSpace:
	LINEAR_SPEED = 0.1
	ANGULAR_SPEED = 0.1
	MIN_SCAN_ANGLE = math.pi + (-20.0 / 180.0 * math.pi)
	MAX_SCAN_ANGLE = math.pi + (20.0 / 180.0 * math.pi)
	MIN_DIST_FROM_OBSTACLE = 0.4


	def __init__(self):
		self.isObstacle = True
		self.isRotation = False
		self.flagOrient = False
		self.flagFirstFreeSpace = True
		self.directionRotation = False
		self.startOrient = 0
		self.targetOrient = 0
		
		self.cmdVelPub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1000)

		self.modelStatesSub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelStatesCallback, queue_size = 1)

		self.scanSub = rospy.Subscriber("/scan", LaserScan, self.scanCallback, queue_size = 1)


	def moveLinear(self):
		msg = Twist()

		msg.linear.x = (-1) * self.LINEAR_SPEED
		self.cmdVelPub.publish(msg)


	def moveAngular(self, direction):
		msg = Twist()

		if direction:
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


	def initStartOrient(self, modelStates):
		if self.flagOrient:
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
		if self.isRotation:
			self.initStartOrient(modelStates)
			curOrient = self.getCurOrient(modelStates)
			deltaOrient = abs(abs(curOrient) - abs(self.startOrient))

			if curOrient >= 0 and self.startOrient <= 0 or curOrient <= 0 and self.startOrient >= 0:
				deltaOrient = 360 - deltaOrient
			if deltaOrient >= self.targetOrient:
				print("modelStatesCallback isRotation")
				self.stop()
				self.isRotation = False


	def setParamsTargetOrient(self, targetIndexStart, targetIndexEnd, justIndex, flag):
		if targetIndexStart == -1 and targetIndexEnd == -1:
			print("setParamsTargetOrient INF ЛИБО ЕСТЬ, ЛИБО НЕТ")
			if not flag: # вообще нет inf
				self.targetOrient = justIndex
				print("setParamsTargetOrient NO INF justIndex =", justIndex)
			else: # все есть inf
				self.targetOrient = 180 # default orientation
				print("setParamsTargetOrient YES INF")
		else:
			print("setParamsTargetOrient targetOrient = (targetIndexStart + targetIndexEnd) / 2.0;")
			self.targetOrient = (targetIndexStart + targetIndexEnd) / 2
			if targetIndexStart >= targetIndexEnd: # если inf находится под индексом 0 или крайний диапазон с inf заканчивается под индексом 0
				print("setParamsTargetOrient если inf находится под индексом 0")
				self.targetOrient = self.targetOrient + 180
				if self.targetOrient >= 360:
					self.targetOrient -= 360
		if self.targetOrient < 180:
			self.targetOrient = 180 - self.targetOrient
			self.directionRotation = False
			print("setParamsTargetOrient targetOrient =", self.targetOrient)
			print("setParamsTargetOrient directionRotation = false")
		else:
			self.targetOrient = self.targetOrient - 180
			self.directionRotation = True
			print("setParamsTargetOrient targetOrient =", self.targetOrient)
			print("setParamsTargetOrient directionRotation = true")
		self.flagOrient = True
		self.isRotation = True
		print("setParamsTargetOrient targetIndexStart =", targetIndexStart)
		print("setParamsTargetOrient targetIndexEnd =", targetIndexEnd)


	def findFreeSpace(self, scan):
		indexStart = -1
		indexEnd = -1
		targetIndexStart = -1
		targetIndexEnd = -1
		loopIndexStart = math.ceil(scan.angle_min / scan.angle_increment)
		loopIndexEnd = math.ceil(scan.angle_max / scan.angle_increment) + 1
		justIndex = -1 # для случая, если вообще не будет inf
		rangeStart = -1
		rangeEnd = -1
		maxRangeStart = -1
		maxRangeEnd = -1
		justMax = -1 # для случая, если вообще не будет inf
		isInf = False
		flag = False # если нашли начало диапазона inf
		i = loopIndexStart
		while i < loopIndexEnd:
			isInf = not (scan.ranges[i] >= scan.range_min and scan.ranges[i] <= scan.range_max)
			if not isInf and scan.ranges[i] > justMax:
				justIndex = i
				justMax = scan.ranges[justIndex]
			if (isInf):
				if not flag:
					indexStart = i - 1
					if indexStart == -1:
						j = loopIndexEnd - 1
						while j > 0:
							if scan.ranges[j] >= scan.range_min and scan.ranges[j] <= scan.range_max:
								indexStart = j
								break
							j -= 1
						if indexStart == -1:
							flag = True
							break
						else:
							loopIndexEnd = indexStart
					rangeStart = scan.ranges[indexStart]
					flag = True
			if flag and (not isInf or i + 1 == loopIndexEnd):
				if not isInf:
					indexEnd = i
				else:
					indexEnd = loopIndexEnd % 360
				rangeEnd = scan.ranges[indexEnd]
				if rangeStart + rangeEnd > maxRangeStart + maxRangeEnd:
					targetIndexStart = indexStart
					targetIndexEnd = indexEnd
					maxRangeStart = rangeStart
					maxRangeEnd = rangeEnd
				elif rangeStart + rangeEnd == maxRangeStart + maxRangeEnd:
					targetDelta = targetIndexEnd - targetIndexStart
					delta = indexEnd - indexStart

					if targetDelta < 0:
						targetDelta = 360 + targetDelta
					if delta < 0:
						delta = 360 + delta
					if delta > targetDelta:
						targetIndexStart = indexStart
						targetIndexEnd = indexEnd
				flag = False
			i += 1
		self.setParamsTargetOrient(targetIndexStart, targetIndexEnd, justIndex, flag)
		print("findFreeSpace ranges")
		i = 0
		while i < 360:
			print("[{%d}] = {%f}", i, scan.ranges[i])
			i += 1


	def scanCallback(self, scan):
		isObstacleInFront = False
		minIndex = math.ceil((self.MIN_SCAN_ANGLE - scan.angle_min) / scan.angle_increment)
		maxIndex = math.ceil((self.MAX_SCAN_ANGLE - scan.angle_min) / scan.angle_increment)

		if (self.flagFirstFreeSpace):
			self.findFreeSpace(scan)
			self.flagFirstFreeSpace = False
		currIndex = minIndex
		while currIndex <= maxIndex:
			if scan.ranges[currIndex] <= self.MIN_DIST_FROM_OBSTACLE:
				isObstacleInFront = True
				break
			currIndex += 1
		if isObstacleInFront:
			self.isObstacle = True
			if not self.isRotation:
				print("scanCallback isObstacle")
				self.stop()
				self.findFreeSpace(scan)
		else:
			self.isObstacle = False


	def startMoving(self):
		rospy.init_node("free_space", anonymous = False)
		rate = rospy.Rate(5000)

		print("Start moving")
		while not rospy.is_shutdown():
			if not self.isObstacle and not self.isRotation:
				self.moveLinear()
			elif self.isRotation and not self.flagOrient:
				self.moveAngular(self.directionRotation)
			rate.sleep()


if __name__ == '__main__':
	try:
		freeSpace = FreeSpace()
		freeSpace.startMoving()
	except rospy.ROSInterruptException:
		pass
