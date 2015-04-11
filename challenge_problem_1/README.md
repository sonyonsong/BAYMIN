# COMS 6998-03 HUMANOID ROBOTS, SPRING 2015
# Columbia University


## Getting Started

The following lines will not work unless ROS is properly installed.  You can find instructions for how to do this from the ros_tutorial pdf on the class website.

```bash
$ cd ~
$ git clone git@github.com:HumanoidRobotics/challenge_problem_1.git
$ cd challenge_problem_1
$ source /opt/ros/hydro/setup.bash
$ catkin_make
$ source devel/setup.bash
```

## Running the Demo code
First, bring up Gazebo, Moveit and the PR2
```bash
$ roslaunch system_launch pr2_gazebo_moveit.launch
```

Then run the individual demos with any of the following:
```bash
$ rosrun move_arm move_arm
$ rosrun move_base move_base
$ rosrun move_gripper move_gripper
$ rosrun move_head move_head
```

##Optional Challenge Problem

Your code will go in either pickup_object.cpp or pickup_object.py in the pickup_object package.  We have provided the
move_* packages to provide code to get you started.  Please make sure that the correct line is uncommented in the pickup_object.launch
file depending on whether you use python or C++.

In order to test your code, please run:
```bash
$ roslaunch system_launch pr2_gazebo_moveit.launch
```

Then:
```bash
$ rosrun pickup_object pickup_object
```
Or if using Python:

```bash
$ rosrun pickup_object pickup_object.py
```

Or you can run everythign at once by running:
```bash
$ roslaunch system_launch challenge_1.launch
```

The goal of this challenge is to have the PR2 navigate to the table, and pick the cup off the table.  It is ok if the cup
falls out of the PR2's gripper after several seconds. This is entirely optional, but a good way to learn about different components of a ROS system.


