#!/usr/bin/env python
# coding: utf-8
import rospy
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

# Convert degrees to radians
DE2RA = pi / 180

if __name__ == '__main__':
    rospy.init_node("set_pose_py", anonymous=True)

    # Initialize MoveIt!
    moveit_commander.roscpp_initialize([])
    
    # 🔹 Change to "ee_group" if necessary
    jetcobot = moveit_commander.MoveGroupCommander("arm_group")  
    
    # ✅ Ensure MoveIt! is using the correct end effector link
    ee_link = jetcobot.get_end_effector_link()
    print(f"🔍 MoveIt! End Effector Link: {ee_link}")

    # If planning with arm_group, explicitly set EE link (change to correct EE name)
    jetcobot.set_end_effector_link("end_effector")  

    # Confirm new end effector link
    print(f"✅ Using End Effector Link: {jetcobot.get_end_effector_link()}")

    # Allow replanning
    jetcobot.allow_replanning(True)
    jetcobot.set_planning_time(5)
    jetcobot.set_num_planning_attempts(10)
    jetcobot.set_goal_tolerance(0.01)

    # Move to initial position
    jetcobot.set_named_target("init")
    jetcobot.go()
    sleep(1)

    # Create a target pose
    target_pose = Pose()
    target_pose.position.x = 0.1
    target_pose.position.y = 0.1
    target_pose.position.z = 0.3

    # Set orientation as a unit quaternion (modify if needed)
    target_pose.orientation.x = 0
    target_pose.orientation.y = 0
    target_pose.orientation.z = 0
    target_pose.orientation.w = 1

    # 🔍 Debugging: Print current pose
    current_pose = jetcobot.get_current_pose().pose
    print(f"📍 Current Pose: {current_pose}")

    # 🔹 Set target pose and plan
    jetcobot.set_pose_target(target_pose)
    plan = jetcobot.plan()

    # ✅ Check if planning was successful
    if plan and plan[0]:
        rospy.loginfo("✅ Plan found! Executing...")
        jetcobot.execute(plan[1])
    else:
        rospy.loginfo("❌ Planning failed. Check kinematics and TF frames.")

    # Shut down MoveIt!
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


