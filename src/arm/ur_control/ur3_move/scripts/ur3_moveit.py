#!/usr/bin/python3
# -*- coding:utf-8 -*-
import sys
import time
import math
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import *
 
 
 
if __name__ == '__main__': 
    rospy.init_node('ur3_moveit')
    moveit_commander.roscpp_initialize(sys.argv)
    arm = moveit_commander.MoveGroupCommander('manipulator')
    
    goal = Pose()
    goal.position.x = 0.
    goal.position.y = 0.
    goal.position.z = 0.
    while goal.position.y < 0.2:
        arm.set_pose_target(goal)
        arm.go(True)
        goal.position.y += 0.04
    moveit_commander.roscpp_shutdown()
