#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
from typing import Final
import math

start = 0

x_lin_pose_start = 0.0
z_ang_pose_start = 0.0

x_lin_pose_current = 0.0
z_ang_pose_current = 0.0

def	pose_callback(pose):
	global start
	global x_lin_pose_start
	global z_ang_pose_start
	global x_lin_pose_current
	global z_ang_pose_current

	if (start == 0):
		x_lin_pose_start = pose.x
		z_ang_pose_start= pose.theta
		start = 1
	x_lin_pose_current = pose.x
	z_ang_pose_current = pose.theta
	rospy.loginfo("x: %.2f, y: %.2f, theta: %.2f\n", pose.x, pose.y, pose.theta)

def	update_pose_start():
	global x_lin_pose_start
	global z_ang_pose_start

	x_lin_pose_start = x_lin_pose_current
	z_ang_pose_start = z_ang_pose_current

def	reset_speed(msg):
	msg.linear.x = 0
	msg.linear.y = 0
	msg.linear.z = 0
	msg.angular.x = 0
	msg.angular.y = 0
	msg.angular.z = 0

def digit_turtle():

	global start

	FORWARD_SPEED_MPS: Final = 3.0

	X_DISTANCE: Final = 3.0

	Z_ROTATION: Final = 3.0 * math.pi / 4.0

	rospy.init_node('digit_turtle', anonymous=False)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	rate = rospy.Rate(100)
	msg = Twist()
	reset_speed(msg)

	rospy.loginfo("Start")

	while not rospy.is_shutdown() and start < 8:
		# 1
		if (start == 1 and (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE)):
			msg.linear.x = FORWARD_SPEED_MPS
		elif (start == 1 and not (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE)):
			update_pose_start()
			reset_speed(msg)
			start = 2

		# 2
		if (start == 2 and (abs(z_ang_pose_current - z_ang_pose_start) < Z_ROTATION)):
			msg.angular.z = -FORWARD_SPEED_MPS
		elif (start == 2 and not (abs(z_ang_pose_current - z_ang_pose_start) < Z_ROTATION)):
			update_pose_start()
			reset_speed(msg)
			start = 3

		# 3
		if (start == 3 and (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE)):
			msg.linear.x = FORWARD_SPEED_MPS
		elif (start == 3 and not (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE)):
			update_pose_start()
			reset_speed(msg)
			start = 4

		# 4
		if (start == 4 and (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE / 2.0)):
			msg.linear.x = -FORWARD_SPEED_MPS
		elif (start == 4 and not (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE / 2.0)):
			update_pose_start()
			reset_speed(msg)
			start = 5

		# 5
		if (start == 5 and (abs(z_ang_pose_current - z_ang_pose_start) < math.pi - Z_ROTATION)):
			msg.angular.z = -FORWARD_SPEED_MPS
		elif (start == 5 and not (abs(z_ang_pose_current - z_ang_pose_start) < math.pi - Z_ROTATION)):
			update_pose_start()
			reset_speed(msg)
			start = 6

		# 6
		if (start == 6 and (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE / 3.0)):
			msg.linear.x = FORWARD_SPEED_MPS
		elif (start == 6 and not (abs(x_lin_pose_current - x_lin_pose_start) < X_DISTANCE / 3.0)):
			update_pose_start()
			reset_speed(msg)
			start = 7

		# 7
		if (start == 7 and (abs(x_lin_pose_current - x_lin_pose_start) < 2.0 * X_DISTANCE / 3.0)):
			msg.linear.x = -FORWARD_SPEED_MPS
		elif (start == 7 and not (abs(x_lin_pose_current - x_lin_pose_start) < 2.0 * X_DISTANCE / 3.0)):
			update_pose_start()
			reset_speed(msg)
			start = 8
		pub.publish(msg)
		rate.sleep()

	msg.linear.x = X_DISTANCE / 3.0
	pub.publish(msg)

	rospy.loginfo("End")

if __name__ == '__main__':
	try:
		digit_turtle()
	except rospy.ROSInterruptException:
		pass