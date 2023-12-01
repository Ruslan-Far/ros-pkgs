#!/usr/bin/env python

from custom_services.srv import *
import rospy

def handle_mean_three_ints(req):
    print("Returning [(%s+%s+%s)/3 = %f]"%(req.a, req.b, req.c, (req.a + req.b +req.c)/3))
    return MeanThreeIntsResponse((req.a + req.b +req.c)/3)

def service_server():
    rospy.init_node('service_server')
    s = rospy.Service('mean_three_ints', MeanThreeInts, handle_mean_three_ints)
    print("Ready to calculate mean of three ints.")
    rospy.spin()

if __name__ == "__main__":
    service_server()
