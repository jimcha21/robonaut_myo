#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *
from gazebo_msgs.srv import *

def add_two_ints_client(x, y):
    #waiting for rosslaunh
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    try:
	h = { 'cookies' : 'peanut butter' } 
	rospy.ServiceProxy('/gazebo/apply_joint_effort', gazebo_msgs.ApplyJointEffort)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
