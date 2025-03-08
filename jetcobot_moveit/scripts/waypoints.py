#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose
from copy import deepcopy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from tf.transformations import quaternion_from_euler


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Initializing the node
    rospy.init_node('cartesian_plan_py')
    scene = PlanningSceneInterface()
    
    # Initialize the robotic arm
    jetcobot = MoveGroupCommander('arm_group')
    # When motion planning fails, re-planning is allowed
    jetcobot.allow_replanning(True)
    jetcobot.set_planning_time(50)
    # Set number of attempts
    jetcobot.set_num_planning_attempts(20)
    # Set target goal and max velocity and acceleration
    jetcobot.set_goal_position_tolerance(0.01)
    jetcobot.set_goal_orientation_tolerance(0.01)
    jetcobot.set_goal_tolerance(0.01)
    jetcobot.set_max_velocity_scaling_factor(1.0)
    jetcobot.set_max_acceleration_scaling_factor(1.0)
    
    # Setting 'init' as the initial pose (home position)
    rospy.loginfo("Home pose")
    jetcobot.set_named_target("init")
    jetcobot.go()
    sleep(0.5)
    
    # Setting target pose
    rospy.loginfo("Setting target pose")
    # # Need to integrate camera info 
    pos = Pose()
    # Back Position
    pos.position.x = -0.226 
    pos.position.y = -0.061
    pos.position.z = 0.3
    pos.orientation.x = -0.008
    pos.orientation.y = -0.591
    pos.orientation.z = -0.004
    pos.orientation.w = 0.807
    # Set the target point
    jetcobot.set_pose_target(pos)
    plan = jetcobot.plan()

    planned_trajectory = plan[1] # Contains the planned trajectory

    # Initialize waypoint list
    waypoints = []
    num_waypoints = 3
    # Get total number of points in trajectory
    total_points = len(planned_trajectory.joint_trajectory.points)
    waypoint_interval = total_points // num_waypoints
    rospy.loginfo(f"Total number of points in the trajectory: {total_points}")
    rospy.loginfo(f"Waypoint interval: {waypoint_interval}")

    for i in range(0 , total_points, waypoint_interval):
        point = planned_trajectory.joint_trajectory.points[i]
        # Set waypoint data and add it to the waypoint list
        waypoints.append(deepcopy(point.positions))
    

    #rospy.loginfo(f"Waypoints: {waypoints}")
    #jetcobot.execute(plan[1])
    sleep(0.5)

    for i in range(len(waypoints)):
        jetcobot.set_joint_value_target(waypoints[i])
        jetcobot.go(wait=True)
        rospy.loginfo(f"Reached waypoint {i}")
        sleep(0.5)
    ''' 
    # Obtain the planned end pose
    planned_end_pos = planned_trajectory.joint_trajectory.points[-1]
    rospy.loginfo(f"Planned end pose: {planned_end_pos}")
    '''

 
    #rospy.spin()
    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
