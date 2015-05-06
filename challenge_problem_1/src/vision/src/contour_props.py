#!/usr/bin/env python
import sys, time
import numpy as np
from scipy.ndimage import filters
import cv2
import roslib
import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray, String
import color_info

VERBOSE=True

class image_feature:

    def __init__(self):
        color_topic = "/color"
        self.color_subscriber = rospy.Subscriber(color_topic,
                String, self.callback_color, queue_size = 1)

        contour_topic = "/output/contour/moments"
        self.publisher = rospy.Publisher(contour_topic,
            Float64MultiArray)

        image_topic = "/wide_stereo/right/image_color/compressed"
        self.subscriber = rospy.Subscriber(image_topic,
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to %s" % image_topic

        # Default color is any non-grey
        self.lower = np.array([0, 50, 0])
        self.higher = np.array([360, 255, 255])

    def callback_color(self, ros_data):
        color = ros_data.data
        print "Changing color to %s" % color

        if color == "blue":
            self.lower = np.array([100, 204, 0])
            self.higher = np.array([160, 255, 255])
        elif color == "green":
            self.lower = np.array([50, 128, 0])
            self.higher = np.array([70, 255, 255])
        elif color == "red":
            self.lower = np.array([0, 204, 0])
            self.higher = np.array([10, 255, 255])
        elif color == "yellow":
            self.lower = np.array([20, 128, 0])
            self.higher = np.array([50, 255, 255])

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        
        time1 = time.time()

        x, area = color_info.color_props(image_np, self.lower, self.higher)
        _, width = image_np.shape[:2]
        x -= width/2
        if VERBOSE :
            print 'blob of centroid %s and area %s' % (x, area)

        time2 = time.time()
        
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)
        
        #rate = rospy.Rate(1) # 1hz 
        self.publisher.publish(data = [float(x), float(area)])
        #rate.sleep()

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
