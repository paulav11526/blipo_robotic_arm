#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from time import sleep
import moveit_commander
from copy import deepcopy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_plan_py')
    scene = PlanningSceneInterface()
    jetcobot = MoveGroupCommander('arm_group')
    # When motion planning fails, re-planning is allowed
    jetcobot.allow_replanning(True)
    jetcobot.set_planning_time(50)
    jetcobot.set_num_planning_attempts(20)
    jetcobot.set_goal_position_tolerance(0.01)
    jetcobot.set_goal_orientation_tolerance(0.01)
    jetcobot.set_goal_tolerance(0.01)
    jetcobot.set_max_velocity_scaling_factor(1.0)
    jetcobot.set_max_acceleration_scaling_factor(1.0)
    rospy.loginfo("reset pose")
    jetcobot.set_named_target("init")
    jetcobot.go()
    sleep(0.5)
    rospy.loginfo("ready pose")
    joints = [-0.07339833069193903, 0.9798838167728141, -2.5519868536114148, 1.57218173439937, 0.07399901831847308, 0.0011697565220774773] #radians 
    jetcobot.set_joint_value_target(joints)
    first_plan = jetcobot.plan()
    jetcobot.execute(first_plan[1])
    sleep(0.5)
    # Obtain the current pose data as the starting pose of the robot arm movement
    end_effector_link = jetcobot.get_end_effector_link()
    start_pose = jetcobot.get_current_pose(end_effector_link).pose
    # Initialize waypoint list
    waypoints = []
    # If True, add the initial pose to the waypoint list
    waypoints.append(start_pose)
    for i in range(3):
        # Set waypoint data and add it to the waypoint list
        wpose = deepcopy(start_pose)
        wpose.position.y -= 0.1
        waypoints.append(deepcopy(wpose))
        wpose.position.y += 0.1
        waypoints.append(deepcopy(wpose))
    # planning process
    fraction = 0.0  # Path planning coverage
    maxtries = 100  # Maximum number of planning attempts
    attempts = 0    # Planning attempts have been made
    rospy.loginfo("Path Planning in Cartesian Space")
    # Try to plan a path in Cartesian space that passes through all waypoints in sequence
    while fraction < 1.0 and attempts < maxtries:
        '''
        waypoints: 路点列表 
        eef_step: 终端步进值，每隔0.1m计算一次逆解判断能否可达
        jump_threshold: 跳跃阈值，设置为0代表不允许跳跃
        plan: 路径, fraction: 路径规划覆盖率
        '''
        (plan, fraction) = jetcobot.compute_cartesian_path(waypoints, 0.1, 0.0, True)
        attempts += 1
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the jetcobot.")
        jetcobot.execute(plan)
        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo("Path planning failed with only " + str(
            fraction) + " success after " + str(maxtries) + " attempts.")
    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

