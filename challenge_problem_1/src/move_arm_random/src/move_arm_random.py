#!/usr/bin/env python

import sys
import rospy
import copy
import IPython
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm_random', anonymous = True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

    rospy.sleep(3)

    print "========= Generating Pose 1 ========="

    target_pose = geometry_msgs.msg.Pose()
    target_pose.orientation.x = 0
    target_pose.orientation.y = -0.629019
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0.77739
    target_pose.position.x = 0.423748
    target_pose.position.y = -0.238729
    target_pose.position.z = 1.20937
    right_arm_group.set_pose_target(target_pose)

    print "========= Moving to Pose 1 ========="

    right_arm_group.go()

    target_pose.orientation.x = 0
    target_pose.orientation.y = -0.629019
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0.77739
    target_pose.position.x = 0.823748
    target_pose.position.y = -0.238729
    target_pose.position.z = 1.20937
    right_arm_group.set_pose_target(target_pose)

    print "========= Generating Cartesian Path =========="

    waypoints = []
    
    # start with current pose
    waypoints.append(right_arm_group.get_current_pose().pose)
    print waypoints[0]
    
    # first move: orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.x = waypoints[0].orientation.x
    wpose.orientation.y = waypoints[0].orientation.y
    wpose.orientation.z = waypoints[0].orientation.z
    wpose.orientation.w = waypoints[0].orientation.w
    wpose.position.x = waypoints[0].position.x
    wpose.position.y = waypoints[0].position.y + 0.5
    wpose.position.z = waypoints[0].position.z

    waypoints.append(copy.deepcopy(wpose))

    # compute cartesian path

    (plan2, fraction) = right_arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    
    rospy.sleep(3)
    print "========= Moving along Cartesian Path =========="
    right_arm_group.go()
    rospy.sleep(3)

    IPython.embed()
