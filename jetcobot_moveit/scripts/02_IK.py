#!/usr/bin/env python
import rospy
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander import RobotCommander, PlanningSceneInterface

DE2RA = pi / 180  # Degrees to Radians conversion

if __name__ == '__main__':
    rospy.init_node("set_pose_py")  # Initialize ROS node

    # Initialize MoveIt! objects
    jetcobot = MoveGroupCommander("arm_group")
    robot = RobotCommander()
    scene = PlanningSceneInterface()

    # MoveIt! Configuration
    jetcobot.allow_replanning(True)
    jetcobot.set_planning_time(10)  # Increase planning time
    jetcobot.set_num_planning_attempts(20)  # Allow more retries
    jetcobot.set_max_velocity_scaling_factor(0.5)  # Slow down motion
    jetcobot.set_max_acceleration_scaling_factor(0.5)  # Reduce acceleration
    jetcobot.set_goal_orientation_tolerance(1.0)

    # Move to initial position
    jetcobot.set_named_target("init")
    jetcobot.go()
    sleep(1)

    jetcobot.set_random_target()
    jetcobot.go()
    sleep(1)
    jetcobot.get_current_pose()
    rospy.loginfo("Current pose: %s", jetcobot.get_current_pose())

    # Define target pose
    pos = Pose()
    pos.position.x = 0.34615096
    pos.position.y = -0.04650332
    pos.position.z = 0.47129924
    pos.orientation.x = 0
    pos.orientation.y = -0.0
    pos.orientation.z = -0.0
    pos.orientation.w = 0.0
    

    if jetcobot.has_end_effector_link():
        ee_link = jetcobot.get_end_effector_link()
        rospy.loginfo(f"End effector: {ee_link}")

        ik_valid = jetcobot.set_pose_target(pos)
        if ik_valid:
            rospy.loginfo("✅ IK solver found a valid solution.")
        else:
            rospy.logwarn("❌ IK solver could NOT find a solution. Target might be unreachable.")

    # Check for self-collisions
    if robot.get_planning_frame() in robot.get_link_names():
        rospy.logwarn("Self-collision detected!")

    # Check if there are any collision objects
    collision_objects = scene.get_known_object_names()
    if collision_objects:
        rospy.logwarn("MoveIt! detects collisions with objects: %s", collision_objects)

    # Try planning
    plan = jetcobot.plan()
    if plan[0]:  # If planning was successful
        rospy.loginfo("Plan successful, executing...")
        jetcobot.execute(plan[1])
    else:
        rospy.logwarn("MoveIt! failed to generate a valid plan!")
    '''
    # 🚨 WARNING: Disabling self-collision checking (for testing)
    jetcobot.set_planner_id("PRMkConfigDefault")  # Use a different planner
    jetcobot.set_path_constraints(None)  # Remove any path constraints
   '''
    # Try planning again
    plan = jetcobot.plan()
    if plan[0]:
        rospy.loginfo("Plan successful, executing...")
        jetcobot.execute(plan[1])
    else:
        rospy.logwarn("Still failing—MoveIt! still sees problems.")

    moveit_commander.roscpp_shutdown()
