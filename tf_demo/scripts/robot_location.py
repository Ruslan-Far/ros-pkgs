#!/usr/bin/env python  
import roslib
import rospy
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
	rospy.init_node('robot_listener')
	listener = tf.TransformListener()
	rate = rospy.Rate(2.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/loaded_map', '/base_footprint', rospy.Time(0))
			print(trans, rot)
			rate.sleep()
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
