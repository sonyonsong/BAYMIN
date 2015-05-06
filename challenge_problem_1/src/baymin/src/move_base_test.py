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
    rospy.init_node('move_base_test', anonymous = True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    

    # OPTION 3: FOLLOW

    dist = float(sys.argv[1])
    p = rospy.Publisher('/base_controller/command', geometry_msgs.msg.Twist)
   
    l = tf.TransformListener()
    l.waitForTransform("/base_footprint", "/odom_combined", rospy.Time(0), rospy.Duration(3.0))
    l.lookupTransform("/base_footprint", "/odom_combined", rospy.Time(0))

    # set up velocity
    goal = geometry_msgs.msg.Twist()
    goal.linear.x = 2.5
    goal.linear.y = 0
    goal.linear.z = 0
    goal.angular.x = 0
    goal.angular.y = 0
    goal.angular.z = 0

    print "TESTING: About to move forward"
    
    # get current x-translation distance
    (trans, rot) = l.lookupTransform("/base_footprint", "/odom_combined", rospy.Time(0))
    curr_dist = trans[0] # x distance
    last_dist = abs(curr_dist)
    dist_moved = 0

    while abs(dist_moved) < abs(dist):
        rate = rospy.Rate(10.0)
        p.publish(goal)
        rate.sleep()

        # get updated distance
        (trans, rot) = l.lookupTransform("/base_footprint", "/odom_combined", rospy.Time(0))
        curr_dist = trans[0]
        delta_dist = abs(curr_dist) - last_dist

        # keep track of how much we've moved forward
        dist_moved += abs(delta_dist)
        last_dist = abs(curr_dist)
        print "dist_moved: ", dist_moved
       
    print "TESTING: Stopping..."
    goal = geometry_msgs.msg.Twist() 
    p.publish(goal)
    

    rospy.signal_shutdown("finished")
