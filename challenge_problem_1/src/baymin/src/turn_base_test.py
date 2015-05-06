#!/usr/bin/env python

import sys
import roscpp
import rospy
import moveit_commander
import tf
import moveit_msgs.msg
import geometry_msgs.msg

if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('turn_base_test', anonymous = True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    

    # OPTION 3: TURN
   
    angle = float(sys.argv[1])
    p = rospy.Publisher('/base_controller/command', geometry_msgs.msg.Twist)

    l = tf.TransformListener()
    l.waitForTransform("/base_footprint", "/odom_combined", rospy.Time(0), rospy.Duration(3.0))  
    l.lookupTransform("/base_footprint", "/odom_combined", rospy.Time(0))

    # set up turn speed
    goal = geometry_msgs.msg.Twist()
    goal.linear.x = 0
    goal.linear.y = 0
    goal.linear.z = 0
    goal.angular.x = 0
    goal.angular.y = 0
    if angle < 0:
        goal.angular.z = 1.5 #clockwise = (-), ccw = (+)
    elif angle > 0:
        goal.angular.z = -1.5        

    print "TESTING: About to turn right"

    # get current angle -- try PyKDL
    (trans, rot) = l.lookupTransform("/base_footprint", "/odom_combined", rospy.Time(0))
    curr_angle = tf.transformations.euler_from_quaternion(rot)[2]
    last_angle = abs(curr_angle)
    angle_turned = 0

    # while turn_angle is less than desired angle
    while abs(angle_turned) < abs(angle):
        rate = rospy.Rate(10.0)
        p.publish(goal)
        rate.sleep()

        # get updated angle
        (trans, rot) = l.lookupTransform("/base_footprint", "/odom_combined", rospy.Time(0))
        curr_angle = tf.transformations.euler_from_quaternion(rot)[2]
        delta_angle = abs(curr_angle) - last_angle
        print "curr_angle: ", curr_angle
        print "delta_angle: ", delta_angle

        # keep track of how much we've turned
        angle_turned += abs(delta_angle)
        last_angle = abs(curr_angle)
        print "angle_turned: ", angle_turned

    print "TESTING: Stoping..."
    goal = geometry_msgs.msg.Twist()
    p.publish(goal)
    

    rospy.signal_shutdown("finished")
