#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import sys
import math

grid = []

def printGridToFile():
    global grid
    rospy.loginfo("Print info to file grid_py.txt")
    fw = open('grid_py.txt', 'w')
    for g in grid:
        for g1 in g:
            if (g1 == True):
                fw.write("1")
            else:
                fw.write("0")
        fw.write("\n")


def requestMap():
    global grid
    rospy.init_node('load_map', anonymous=False)
    rospy.wait_for_service('static_map')
    try:
        static_map = rospy.ServiceProxy("static_map", GetMap)
        map = static_map().map
        rows = map.info.height
        cols = map.info.width
        rospy.loginfo("rows=%i, cols=%i", rows, cols)
        currCell = 0
        i = 0
        while i < rows:
            j = 0
            grid.append([])
            while j < cols:
                if (map.data[currCell] == 0):
                    grid[i].append(False)
                else:
                    grid[i].append(True)
                j = j + 1
                currCell = currCell + 1
            i = i + 1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    rospy.loginfo("Write to grid finished")

if __name__ == '__main__':
    try:
        requestMap()
        printGridToFile()
    except rospy.ROSInterruptException:
        pass

