#!/usr/bin/env python
# -*- coding: utf-8 -*-


# To have an inital camera reading and go to it

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
        rospy.init_node('set_camera_plan', anonymous=True)
        scene = PlanningSceneInterface()

        self.pos = Pose()
        self.received_coordinates = False # Flag to check if coordinates have been received
        
        # Subscribing to base coordinates
        rospy.Subscriber("base_coordinates", Point, self.base_callback)

        # Initialize the robotic arm
        self.jetcobot = MoveGroupCommander('arm_group')
        # When motion planning fails, re-planning is allowed
        self.jetcobot.allow_replanning(True)
        self.jetcobot.set_planning_time(5)
        # Set number of attempts
        self.jetcobot.set_num_planning_attempts(10)
        # Set target goal and max velocity and acceleration
        self.jetcobot.set_goal_position_tolerance(0.001)
        self.jetcobot.set_goal_orientation_tolerance(0.01)
        self.jetcobot.set_goal_tolerance(0.001)
        self.jetcobot.set_max_velocity_scaling_factor(1.0)
        self.jetcobot.set_max_acceleration_scaling_factor(1.0)
        
        '''# Setting 'vertical' as the initial position
        rospy.loginfo("Vertical Surfaces")
        self.jetcobot.set_named_target("vertical")
        self.jetcobot.go()
        sleep(0.5)'''
        
        # Setting 'bottom vertical' position
        rospy.loginfo("Bottom vertical position")
        self.jetcobot.set_named_target("bottom vertical")
        self.jetcobot.go()
        sleep(0.5)


    def base_callback(self, msg):
        # Store received base coordinates
        #rospy.loginfo("Base coordinates received")
        self.pos.position.x = msg.x
        self.pos.position.y = msg.y
        self.pos.position.z = msg.z
        self.xyz = [self.pos.position.x, self.pos.position.y, self.pos.position.z]
        self.received_coordinates = True

    def wait_for_coordinates(self):
        rospy.loginfo("Waiting for base coordinates...")
        while not self.received_coordinates:
            rospy.sleep(0.1)

    def plan_pose(self):
        # Set the target point
        rospy.loginfo("Setting target position")
        self.jetcobot.set_position_target(self.xyz)
        rospy.loginfo(f"Setting position with coordinates: {self.pos}")
        plan = self.jetcobot.plan()
        rospy.loginfo(f"Planned trajectory: {plan}")

        if not plan or not plan[0]:
            rospy.logwarn("Motion planning failed.")
            return None

        return plan[1]
    
    def execute_motion(self):
        while not rospy.is_shutdown():
            self.wait_for_coordinates()
            plan = self.plan_pose()
            if plan:
                self.jetcobot.execute(plan)
                rospy.loginfo("Motion executed successfully.")
                rospy.sleep(3)
                # Reset the flag to wait for new coordinates
                self.received_coordinates = False
                rospy.loginfo("Bottom vertical position")
                self.jetcobot.set_named_target("bottom vertical")
                self.jetcobot.go()
                sleep(3)

                #break
            else:
                rospy.logwarn("Motion planning failed. Retrying...")
                self.received_coordinates = False  # Reset the flag to wait for new coordinates

if __name__ == "__main__":
    run = Waypoints()
    run.execute_motion()
    rospy.spin()
