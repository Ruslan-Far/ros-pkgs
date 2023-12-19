#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from semester_work.srv import *

cleanedMap = OccupancyGrid()

def duplicateCleanedMap(cleanedMapFromSrv):
    global cleanedMap
    
    cleanedMap.header.seq = cleanedMapFromSrv.header.seq
    cleanedMap.header.stamp = cleanedMapFromSrv.header.stamp
    cleanedMap.header.frame_id = cleanedMapFromSrv.header.frame_id
    
    cleanedMap.info.map_load_time = cleanedMapFromSrv.info.map_load_time
    cleanedMap.info.resolution = cleanedMapFromSrv.info.resolution
    cleanedMap.info.width = cleanedMapFromSrv.info.width
    cleanedMap.info.height = cleanedMapFromSrv.info.height
    cleanedMap.info.origin.position.x = cleanedMapFromSrv.info.origin.position.x
    cleanedMap.info.origin.position.y = cleanedMapFromSrv.info.origin.position.y
    cleanedMap.info.origin.position.z = cleanedMapFromSrv.info.origin.position.z
    cleanedMap.info.origin.orientation.x = cleanedMapFromSrv.info.origin.orientation.x
    cleanedMap.info.origin.orientation.y = cleanedMapFromSrv.info.origin.orientation.y
    cleanedMap.info.origin.orientation.z = cleanedMapFromSrv.info.origin.orientation.z
    cleanedMap.info.origin.orientation.w = cleanedMapFromSrv.info.origin.orientation.w

    i = 0
    while i < len(cleanedMapFromSrv.data):
        cleanedMap.data.append(cleanedMapFromSrv.data[i])
        i += 1


def mapCallback(map, client):
    print("Waiting for service cleaned_map to become available")
    rospy.wait_for_service("cleaned_map", rospy.Duration(33))
    try:
         res = client(map)
         duplicateCleanedMap(res.cleanedMap)
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
    