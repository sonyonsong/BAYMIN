#!/usr/bin/env python

import rospy
import numpy as np
import tf
from roslib import message
from std_msgs.msg import Float64
import sensor_msgs.point_cloud2 as pcl
from sensor_msgs.msg import PointCloud2
from perception_msgs.msg import SegmentedObject, SegmentedObjectList

def callback_kinect(data):
    
    global pcl_sub
    global depth_pub

    print "callback"
    min_depth = float("inf")
    l = tf.TransformListener()

    seg_obj_list = data.segmentedObjects
    for seg_obj in seg_obj_list:
        print "=============NEW OBJECT============="
        cloud = seg_obj.segmentedObjectPointCloud
        # below line doesn't work!
        # new_cloud = l.transformPointCloud('/r_wrist_roll_joint',cloud)
        # print "cloud: ", cloud
        
        data_out = pcl.read_points(cloud, skip_nans=False)
        points = []
        for pt in data_out:
            pt = list(pt)
            print pt
            x = pt[0]
            y = pt[1]
            z = pt[2]

            if z < min_depth and z > 1.0:
               min_depth = z
            points.append(pt)


    # pcl_sub.unregister()
    print min_depth
    # guesstimated offset because there's not transformPointCloud function in python for PointCloud2!
    offset = 0.6
    #rate = rospy.Rate(1.0)
    if min_depth != float("inf"):
        depth_pub.publish(min_depth - offset)
    else:
        depth_pub.publish(None)
    #rate.sleep()
    rospy.sleep(1)
    


def listen():

    global pcl_sub
    global depth_pub

    rospy.init_node('listen', anonymous=True)
    
    pcl_sub = rospy.Subscriber("segmented_objects", SegmentedObjectList, callback_kinect, queue_size = 10)
    print "subsribed to segmented_objects"
    
    depth_pub = rospy.Publisher("/bump_depth", Float64)
    print "ready to publish to /bump_depth"

    rospy.spin()


if __name__ == '__main__':

    listen()
