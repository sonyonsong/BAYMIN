#!/usr/bin/env python

import rospy
from tuck_arms.srv import *
from pr2_tuck_arms_action import tuck_arms_main

def tuck_cb(req):
    if req.tuck == True:
        tuck_arms_main.main()
    print "tucking"
    return True

def tuck_arms_server():
    rospy.init_node("arm_tucking_service")
    s = rospy.Service("arm_tucking_service", Tuck, tuck_cb)
    print "service running"
    rospy.spin()

if __name__ == '__main__':
    tuck_arms_server()
