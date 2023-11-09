#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys
import math


class Stopper:
	FORWARD_SPEED = 0.1
	MIN_SCAN_ANGLE = math.pi - (15.0/180.0*math.pi)
	MAX_SCAN_ANGLE = math.pi + (15.0/180.0*math.pi)
	MIN_DIST_FROM_OBSTACLE = 0.6
	keepMoving = True

	def __init__(self):
		pass

	def start_move(self):
		rospy.init_node('stopper', anonymous=False)
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		sub = rospy.Subscriber('scan', LaserScan, self.callback)
		rate = rospy.Rate(10) # 10hz
		vel = Twist()
		self.keepMoving = True
		while not rospy.is_shutdown():
			if self.keepMoving:
				vel.linear.x = (-1)*self.FORWARD_SPEED
				pub.publish(vel)
				rate.sleep()
			else:
				vel.linear.x = 0
				pub.publish(vel)
				rate.sleep()
	
	def callback(self, msg):
		isObstacleInFront = False
		minIndex = round((self.MIN_SCAN_ANGLE - msg.angle_min) / msg.angle_increment)
		maxIndex = round((self.MAX_SCAN_ANGLE - msg.angle_min) / msg.angle_increment)
		currIndex = minIndex + 1
		while currIndex <= maxIndex:
			if msg.ranges[int(currIndex)] < self.MIN_DIST_FROM_OBSTACLE:
				isObstacleInFront = True
				break
			currIndex = currIndex + 1
		if isObstacleInFront:
			print("Stop!")
			self.keepMoving = False;
		else:
			self.keepMoving = True;
			

if __name__ == '__main__':
	try:
		st = Stopper()
		st.start_move()
	except rospy.ROSInterruptException:
		pass
