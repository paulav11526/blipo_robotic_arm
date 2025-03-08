#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_visual_tools
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander import RobotCommander, PlanningSceneInterface

# Initialize ROS node
rospy.init_node("blipo_workspace_visualizer")

# Initialize MoveIt commander
robot = RobotCommander()
scene = PlanningSceneInterface()
group_name = "arm"  # Change this if needed
move_group = moveit_commander.MoveGroupCommander(group_name)

# Initialize MoveIt! Visual Tools
rviz_visual_tools = moveit_visual_tools.MoveItVisualTools("base_link")  # Base frame
rviz_visual_tools.deleteAllMarkers()
rviz_visual_tools.loadRemoteControl()

# Generate random workspace points
for _ in range(50):  # Adjust to show more or fewer points
    move_group.set_random_target()
    pose = move_group.get_current_pose().pose

    # Visualize with a small sphere
    rviz_visual_tools.publishSphere(pose.position, moveit_visual_tools.GREEN, 0.02)  # 2 cm sphere
    rospy.sleep(0.05)

rviz_visual_tools.trigger()
rospy.loginfo("Workspace visualization complete!")
rospy.spin()

