import rospy
from datetime import datetime

def print_current_date_time():
	rospy.init_node('timer_node', anonymous=False)
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		current_date_time = datetime.now()
		rospy.loginfo("%d:%d %d.%d.%d"
		% (current_date_time.hour, current_date_time.minute, current_date_time.day, current_date_time.month, current_date_time.year))
		rate.sleep()

if __name__ == "__main__":
	try:
		print_current_date_time()
	except rospy.ROSInterruptException: 
		pass
