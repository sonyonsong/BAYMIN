#!/usr/bin/env python

import rospy


if __name__ == '__main__':

    #initialize the node
    rospy.init_node('pickup_object', anonymous=True)

    #sleep for a moment while everything starts up
    rospy.sleep(1.0)

    # your code here:
    rospy.loginfo("Finished Picking up the cup!")