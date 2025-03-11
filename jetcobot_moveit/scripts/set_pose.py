#!/usr/bin/env python
# coding: utf-8

### This code has been modified from the original 02_set_pose_plan.py from the Yahboom repository

import rospy
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler

# degrees to radians
DE2RA = pi / 180

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node("set_pose_py")
    # Initialize the robot arm
    jetcobot = MoveGroupCommander("arm_group")
    # When motion planning fails, re-planning is allowed
    jetcobot.allow_replanning(True)
    jetcobot.set_planning_time(5)
    # Number of planning attempts
    jetcobot.set_num_planning_attempts(10)
    # Set the allowable error of position (unit: meters) and attitude (unit: radians)
    jetcobot.set_goal_position_tolerance(0.01)
    jetcobot.set_goal_orientation_tolerance(0.01)
    # Set allowable target error
    jetcobot.set_goal_tolerance(0.01)
    # Set maximum allowed speed and acceleration
    jetcobot.set_max_velocity_scaling_factor(0.1)
    jetcobot.set_max_acceleration_scaling_factor(0.1)
    # Set "init" as the initial point
    jetcobot.set_named_target("init")
    jetcobot.go()
    sleep(1)
    # Create pose instance
    pos = Pose()
    '''
    # Back
    # Position
    pos.position.x = -0.226 
    pos.position.y = -0.061
    pos.position.z = 0.3
    # Orientation
    pos.orientation.x = -0.008
    pos.orientation.y = -0.591
    pos.orientation.z = -0.004
    pos.orientation.w = 0.807
    # Setting target
    jetcobot.set_pose_target(pos)
    plan = jetcobot.plan()
    # print("plan = ",plan)
    if plan[0]==True:
        rospy.loginfo("plan success")
        # If plan is a success then execute the path
        jetcobot.execute(plan[1])
    else:
        rospy.loginfo("plan error")
        
    '''
        
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    sleep(1)
