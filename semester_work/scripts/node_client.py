#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from semester_work.srv import *

cleanedMap: OccupancyGrid

def mapCallback(map, client):
    while (not rospy.wait_for_service("cleaned_map", rospy.Duration(3))):
        print("Waiting for service cleaned_map to become available")
    try:
         global cleanedMap
         req = CleanedMapRequest(map)
         res = CleanedMapResponse()
         a = client(req, res)
         print(a)
         cleanedMap = res.cleanedMap
         print(cleanedMap)
    except rospy.ServiceException as e:
        print("Failed to call service")


def run_process():
    rospy.init_node("node_client", anonymous=False)
    client = rospy.ServiceProxy("cleaned_map", CleanedMap)
    newMapPub = rospy.Publisher("/new_map", OccupancyGrid, queue_size=1)
    mapSub = rospy.Subscriber("/map", OccupancyGrid, lambda map: mapCallback(map, client), queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
         newMapPub.publish(cleanedMap)
         rate.sleep()


if __name__ == '__main__':
    try:
        run_process()
    except rospy.ROSInterruptException:
        pass
    