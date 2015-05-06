#!/usr/bin/env python

import sys
sys.path.append("/home/esl2131/wit/pywit")
sys.path.append("/home/esl2131/wit/challenge_problem_1/src/tuck_arms/")
import wit_script
import roscpp
import rospy
import IPython
import moveit_commander
import tf
import time
from tuck_arms.srv import *
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Float64MultiArray, Float64


# global variables
VERBOSE = True
REST = 4
ACTION = REST
MOVE_FORWARD = 1
TURN_LEFT = 2
TURN_RIGHT = 3
MOVE_AMT = 0.2 # approx 50 cm
TURN_AMT = 0.1745 # approx 10 deg (CW: +, CCW: -)
angle = 0
dist = 0
isIdle = False
target_area = float(0.0)
threshold = 50 #pixels for noise
isTargetSet = False
rest_count = 0 # how many times area has stayed the same 
isMoving = False
isBumping = False
isBumpDone = False



def bump(pcl_data):
   
    global isBumping
    global isBumpDone
    global target
    global right_arm_group

   
    isBumping = True
    print "depth: ", pcl_data.data

    if pcl_data.data > 0.1:
        move_forward(pcl_data.data - 0.1)
        depth = 0.1
    else:
        depth = pcl_data.data

    
    target_stamped = right_arm_group.get_current_pose()
    target = target_stamped.pose

    print "move gripper +x"
    target.position.x += depth
    right_arm_group.set_pose_target(target)
    right_arm_group.go()

    rospy.sleep(2)

    target_stamped = right_arm_group.get_current_pose()
    target = target_stamped.pose

    print "move gripper -x"
    target.position.x -= depth
    right_arm_group.set_pose_target(target)
    right_arm_group.go()

    rospy.sleep(2)

    target_stamped = right_arm_group.get_current_pose()
    target = target_stamped.pose

    print "restore gripper"
    target.orientation.x = 0.000143
    target.orientation.y = -0.6289
    target.orientation.z = -0.000248
    target.orientation.w = 0.7774
    right_arm_group.set_pose_target(target)
    right_arm_group.go() 
   
    rospy.sleep(2)   
    isBumpDone = True
    #rospy.sleep(2)


    
def compute_direction(ros_data):
    
    global isMoving
    if isMoving:
        return

    [centroid, area] = ros_data.data
    print "centroid: ", centroid
    print "area: ", area
    
    global isTargetSet
    global target_area
    global ACTION
    global angle
    global dist
    global rest_count
    global isIdle
    global start_time
   
    if centroid < -threshold:
        print "========== Turning Left... ========="
        ACTION = TURN_LEFT
        angle = -TURN_AMT
        if centroid > -threshold*2:
            angle = -TURN_AMT/4
        if centroid > -threshold/2 and centroid < 0:
            ACTION = REST
            return
        turn(angle)
    elif centroid > threshold:
        print "========= Turning Right... ========"
        ACTION = TURN_RIGHT
        angle = TURN_AMT
        if centroid < threshold*2:
            angle = TURN_AMT/4
        if centroid < threshold/2 and centroid > 0:
            ACTION = REST
            return
        turn(angle)
    elif isTargetSet == False:
        print "========= Setting Target Size... ========="
        if area > 100:
            target_area = area
            isTargetSet = True
            startTime = time.time()
        # check if area is less than a certain amount
        # turn to find the target and set the target_area
        else:
            print "========== Searching for Person... ==========="
            ACTION = TURN_RIGHT
            angle = TURN_AMT
            turn(angle)
    else:
        if area > 0.8*target_area and area < 1.2*target_area:
            print "========== No Movement Detected... ========="
            ACTION = REST           
            if (time.time() - start_time) > 30:
                isIdle = True
                start_time = time.time()
        elif area < 0.8*target_area and area > 100:
            print "========= Moving Towards Person... ========="
            ACTION = MOVE_FORWARD
        elif area < 100:
            isTargetSet = False


def move_forward(dist):

    print "========== Moving Forward... ========="

    p = rospy.Publisher('/base_controller/command', geometry_msgs.msg.Twist)
       
    l = tf.TransformListener()
    l.waitForTransform("/base_footprint", "/odom_combined", rospy.Time(0), rospy.Duration(3.0))
    l.lookupTransform("/base_footprint", "/odom_combined", rospy.Time(0))
                   
    # set up velocity
    goal = geometry_msgs.msg.Twist()
    goal.linear.x = 5
    goal.linear.y = 0
    goal.linear.z = 0
    goal.angular.x = 0
    goal.angular.y = 0
    goal.angular.z = 0
                                                           
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
        #print "dist_moved: ", dist_moved
                                                      
    print "========= Stopping... =========="
    goal = geometry_msgs.msg.Twist() 
    p.publish(goal)


def turn(angle):

    print "========= Turning... =========="
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

    # get current angle
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
        
        # keep track of how much we've turned
        angle_turned += abs(delta_angle)
        last_angle = abs(curr_angle)
        #print "angle_turned: ", angle_turned

    print "========= Stopping... ========="
    goal = geometry_msgs.msg.Twist()
    p.publish(goal)










def main(argv):

    global right_arm_group
    global target


    # initize nodes, publishers, MoveIt modules, etc.
    moveit_commander.roscpp_initialize(argv)
    rospy.init_node('baymin', anonymous = True)
    color_pub = rospy.Publisher("/color", String)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
    left_arm_group = moveit_commander.MoveGroupCommander("left_arm")

    rospy.sleep(1)

    #print "========= Tuck Both Arms ========="

    #rospy.wait_for_service('arm_tucking_service')
    #t = rospy.ServiceProxy('arm_tucking_service', Tuck)
    #response = t(True)
    #print response

    print "========= Tuck Left Arm =========="

    tuck_left = geometry_msgs.msg.Pose()
    tuck_left.orientation.x = 0.477
    tuck_left.orientation.y = -0.491
    tuck_left.orientation.z = 0.54
    tuck_left.orientation.w = -0.48
    tuck_left.position.x = 0.12
    tuck_left.position.y = -0.114
    tuck_left.position.z = 0.384
    left_arm_group.set_pose_target(tuck_left)
    left_arm_group.go()

    rospy.sleep(3)

    print "========= Set Up Right Arm ========="

    target = geometry_msgs.msg.Pose()
    target.orientation.x = 0.000143
    target.orientation.y = -0.6289
    target.orientation.z = -0.000248
    target.orientation.w = 0.7774
    target.position.x = 0.423696
    target.position.y = -0.2387
    target.position.z = 1.209
    right_arm_group.set_pose_target(target)
    right_arm_group.go()

    rospy.sleep(3)

    ## CHECK FOR VOICE COMMAND

    listen = True

    while listen == True:

        rospy.sleep(1)
        (action, attribute) = wit_script.wit_start()
        #action = "bump"
        #attribute = "red"

        if action == "wave":

            ## OPTION 1: GREETING

            print "========== Waving... ========="
            
            target_stamped = right_arm_group.get_current_pose()
            target = target_stamped.pose
            target.position.y += 0.1
            right_arm_group.set_pose_target(target)
            right_arm_group.go()

            for i in range(2):

                target.position.y -= 0.2
                right_arm_group.set_pose_target(target)
                right_arm_group.go()

                target.position.y += 0.2
                right_arm_group.set_pose_target(target)
                right_arm_group.go()

            target.position.y -= 0.1
            right_arm_group.set_pose_target(target)
            right_arm_group.go()

        elif action == "bump":

            # OPTION 2: FIST BUMP
            global isBumping
            global isBumpDone
            
            while isBumpDone == False:
                if isBumping == False:

                    print "========== Bumping... ========="
                    target_stamped = right_arm_group.get_current_pose()
                    target = target_stamped.pose
                    target.orientation.w += 0.1
                    right_arm_group.set_pose_target(target)
                    right_arm_group.go()
                    rospy.sleep(2)


                    depth_sub = rospy.Subscriber("/bump_depth", Float64, bump, queue_size=1)            
                    if VERBOSE :
                        print "subscribed to /bump_depth"

                    #rospy.sleep(1)
          
          
            depth_sub.unregister()
            isBumpDone = False
            isBumping = False
          

     

        elif action == "follow":

            # OPTION 3: FOLLOW
            target_stamped = right_arm_group.get_current_pose()
            target = target_stamped.pose
            target.position.z -= 0.3
            right_arm_group.set_pose_target(target)
            right_arm_group.go()


            print "========= Following Person in %s ==========" % attribute

            # publish the color so the vision module can read it
            color_pub.publish(attribute)
            if VERBOSE:
                print "published the color %s" % (attribute)
            
            global start_time
            start_time = time.time()

            # create subscriber for vision detection & listen for output
            color_info_sub = rospy.Subscriber("/output/contour/moments", Float64MultiArray, compute_direction, queue_size=1)
            if VERBOSE :
                print "subscribed to /output/contour/moments"  
            
            global isIdle
            global isTargetSet
            while isIdle == False:
             
                if isTargetSet == True:
                   
                    # move accordingly
                    global isMoving
                    isMoving = True
                    if ACTION == MOVE_FORWARD:
                        move_forward(MOVE_AMT)
                    if ACTION == TURN_LEFT or ACTION == TURN_RIGHT:
                        turn(TURN_AMT)
                    isMoving = False
                    rospy.sleep(1)

            color_pub.publish("none")
            color_info_sub.unregister()
            isTargetSet = False
            isIdle = False
            rest_count = 0

            target_stamped = right_arm_group.get_current_pose()
            target = target_stamped.pose
            target.position.z += 0.3
            right_arm_group.set_pose_target(target)
            right_arm_group.go()
 

        elif action == "stop":

            listen = False

        else:
            
           print " VOICE COMMAND NOT IN DATABASE! "

    print "=========== FINISHED ALL ACTION REQUESTS ============ "
    #IPython.embgd()
    #rospy.signal_shutdown("finished")

if __name__ == '__main__':
    main(sys.argv)
