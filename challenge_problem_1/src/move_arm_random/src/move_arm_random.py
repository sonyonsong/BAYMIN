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
    left_arm_group = moveit_commander.MoveGroupCommander("left_arm")

    rospy.sleep(3)

    print "========= Tuck Left Arm =========="

    pose = geometry_msgs.msg.Pose()
    pose.orientation.x = 0.478
    pose.orientation.y = -0.491
    pose.orientation.z = 0.547
    pose.orientation.w = -0.47
    pose.position.x = 0.118
    pose.position.y = -0.113
    pose.position.z = 0.384
    left_arm_group.set_pose_target(pose)
    left_arm_group.go()

    ## CHECK FOR VOICE COMMAND HERE

    ## OPTION 1: GREETING

    print "========= Moving to Pose 1 ========="

    target_pose = geometry_msgs.msg.Pose()
    target_pose.orientation.x = 0
    target_pose.orientation.y = -0.629019
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0.77739
    target_pose.position.x = 0.423748
    target_pose.position.y = -0.238729
    target_pose.position.z = 1.20937
    right_arm_group.set_pose_target(target_pose)
    right_arm_group.go()

    print "========== Waving... ========="

    target_pose.position.y += 0.1
    right_arm_group.set_pose_target(target_pose)
    right_arm_group.go()

    for i in range(2):

        target_pose.position.y -= 0.2
        right_arm_group.set_pose_target(target_pose)
        right_arm_group.go()

        target_pose.position.y += 0.2
        right_arm_group.set_pose_target(target_pose)
        right_arm_group.go()

    target_pose.position.y -= 0.1
    right_arm_group.set_pose_target(target_pose)
    right_arm_group.go()
    
    # OPTION 2: FIST BUMP

    print "========= Moving to Pose 2 =========="
    
    target_pose.orientation.x = 0
    target_pose.orientation.y = -0.629019
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0.97739
    target_pose.position.x = 0.423748
    target_pose.position.y = -0.238729
    target_pose.position.z = 1.20937
    right_arm_group.set_pose_target(target_pose)
    right_arm_group.go()

    print "========== Bumping... ========="

    target_pose.position.x += 0.1
    right_arm_group.set_pose_target(target_pose)
    right_arm_group.go()

    target_pose.position.x -= 0.1
    right_arm_group.set_pose_target(target_pose)
    right_arm_group.go()

    #IPython.embed()
