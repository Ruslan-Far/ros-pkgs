#!/usr/bin/env python

import sys
import rospy
from custom_services.srv import *

def service_client(x, y, z):
    rospy.wait_for_service('mean_three_ints')
    try:
        mean_three_ints = rospy.ServiceProxy('mean_three_ints', MeanThreeInts)
        resp = mean_three_ints(x, y, z)
        return resp.mean
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        z = int(sys.argv[3])
    else:
        print(usage())
        sys.exit(1)

    print("Requesting %s %s %s"%(x, y, z))
    print("(%s+%s+%s)/3 = %s"%(x, y, z, service_client(x, y, z)))
