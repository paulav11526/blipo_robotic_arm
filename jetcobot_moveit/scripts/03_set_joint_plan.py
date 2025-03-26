#!/usr/bin/env python
# coding: utf-8
import rospy
from math import pi
from time import sleep
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler

# Converting from degrees to radians
DE2RA = pi / 180

if __name__ == '__main__':
    # Initializing node
    rospy.init_node("set_joint_py", anonymous=True)
    # Initialize the robotic arm
    jetcobot = MoveGroupCommander("arm_group")
    # When motion planning fails, re-planning is allowed
    jetcobot.allow_replanning(True)
    jetcobot.set_planning_time(5)
    # Number of attemps to plan
    jetcobot.set_num_planning_attempts(10)
    # Sets the allowable target angle error
    jetcobot.set_goal_joint_tolerance(0.001)
    # Sets the maximum allowed speed and acceleration
    jetcobot.set_max_velocity_scaling_factor(1.0)
    jetcobot.set_max_acceleration_scaling_factor(1.0)
    # Set "init" as the initial point (the home position)
    jetcobot.set_named_target("init")
    jetcobot.go()
    sleep(0.5)
    # Set the target point radian
    joints = [1.567, -1.857, -1.069, -0.08, 0.0, 0.0]
    jetcobot.set_joint_value_target(joints)
    # Execute multiple times to improve the success rate
    for i in range(5):
        # Exercise planning
        plan = jetcobot.plan()
        # print("plan = ",plan)
        if plan[0]==True:
            rospy.loginfo("plan success")
            # Run after the plan is successful
            jetcobot.execute(plan[1])
            break
        else:
            rospy.loginfo("plan error")
