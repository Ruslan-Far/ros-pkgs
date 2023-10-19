#!/usr/bin/env python
import rospy

def func() :
	rospy.init_node('my_node', anonymous=True)
	rate = rospy.Rate(1)
	count = 0
	while not rospy.is_shutdown():
		hello_str = "--------py hello world %s" % count
		count = count + 1
		rospy.loginfo(hello_str)
		rate.sleep()

if __name__ == "__main__":
	try:
		func()
	except rospy.ROSInterruptException: 
		pass
