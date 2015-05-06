#!/usr/bin/evn python

import rospy
from tuck_arms.srv import *

if __name__ == "__main__":
    rospy.wait_for_service('arm_tucking_service')
    t = rospy.ServiceProxy('arm_tucking_service', Tuck)
    response = t(True)
    print response

