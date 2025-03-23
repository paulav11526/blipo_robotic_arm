#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from copy import deepcopy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from tf.transformations import quaternion_from_euler

class Waypoints:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initializing the node
        rospy.init_node('cartesian_plan_py')
        scene = PlanningSceneInterface()
        
        # Subscribing to base coordinates
        rospy.Subscriber("base_coordinates", Point, self.base_callback)

        # Initialize the robotic arm
        self.jetcobot = MoveGroupCommander('arm_group')
        # When motion planning fails, re-planning is allowed
        self.jetcobot.allow_replanning(True)
        self.jetcobot.set_planning_time(50)
        # Set number of attempts
        self.jetcobot.set_num_planning_attempts(20)
        # Set target goal and max velocity and acceleration
        self.jetcobot.set_goal_position_tolerance(0.01)
        self.jetcobot.set_goal_orientation_tolerance(0.01)
        self.jetcobot.set_goal_tolerance(0.01)
        self.jetcobot.set_max_velocity_scaling_factor(1.0)
        self.jetcobot.set_max_acceleration_scaling_factor(1.0)
        
        # Setting 'init' as the initial pose (home position)
        self.pos = Pose()
        rospy.loginfo("Home pose")
        self.jetcobot.set_named_target("init")
        self.jetcobot.go()
        sleep(0.5)

    def base_callback(self, msg):
        # Set the target point
        rospy.loginfo("Setting target pose")
        self.pos.position.x = msg.x
        self.pos.position.y = msg.y
        self.pos.position.z = msg.z
        self.pos.orientation.x = 0.0
        self.pos.orientation.y = 0.0
        self.pos.orientation.z = 0.0
        self.pos.orientation.w = 1.0

    
    def plan_pose(self, pose):
        # Set the target point
        rospy.loginfo("Setting target pose")
        self.jetcobot.set_pose_target(pose)
        plan = self.jetcobot.plan()

        planned_trajectory = plan[1] # Contains the planned trajectory
        return planned_trajectory 
    
    def plan_waypoints(self, trajectory):
        # Initialize waypoint list
        waypoints = []
        num_waypoints = 3
        # Get total number of points in trajectory
        total_points = len(trajectory.joint_trajectory.points)
        waypoint_interval = total_points // num_waypoints
        rospy.loginfo(f"Total number of points in the trajectory: {total_points}")
        rospy.loginfo(f"Waypoint interval: {waypoint_interval}")

        for i in range(0 , total_points, waypoint_interval):
            point = trajectory.joint_trajectory.points[i]
            # Set waypoint data and add it to the waypoint list
            waypoints.append(deepcopy(point.positions))
        

        #rospy.loginfo(f"Waypoints: {waypoints}")     
        return waypoints   

    def execute(self, waypoints):
        for i in range(len(waypoints)):
            self.jetcobot.set_joint_value_target(waypoints[i])
            self.jetcobot.go(wait=True)
            rospy.loginfo(f"Reached waypoint {i}")
            sleep(0.5)


if __name__ == "__main__":
    run = Waypoints()
    trajectory = run.plan_pose(run.self.pos)
    waypoints = run.plan_waypoints(trajectory)
    run.execute(waypoints)



    #rospy.spin()
    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
