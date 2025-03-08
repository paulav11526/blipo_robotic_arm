#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import pi
import rospy, rospkg
from time import sleep
import moveit_commander
from moveit_msgs.msg import PlanningSceneWorld
from jetcobot_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, PlanningScene, PlannerInterfaceDescription
from sensor_msgs.msg import JointState


def add_obj(table_pose, obj, table_size, xyz):
    table_pose.header.frame_id = 'base_link'
    table_pose.pose.position.x = xyz[0]
    table_pose.pose.position.y = xyz[1]
    table_pose.pose.position.z = xyz[2]
    table_pose.pose.orientation.w = 1.0
    scene.add_box(obj, table_pose, table_size)

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Set_Scene')
    jetcobot = MoveGroupCommander('arm_group')
    end_effector_link = jetcobot.get_end_effector_link()
    print("end_effector_link = ",end_effector_link)
    scene = PlanningSceneInterface()
    scene.remove_attached_object(end_effector_link, "tool")
    scene.remove_world_object()
    jetcobot.allow_replanning(True)
    jetcobot.set_planning_time(20)
    jetcobot.set_num_planning_attempts(10)
    jetcobot.set_goal_position_tolerance(0.01)
    jetcobot.set_goal_orientation_tolerance(0.01)
    jetcobot.set_goal_tolerance(0.01)
    jetcobot.set_max_velocity_scaling_factor(1.0)
    jetcobot.set_max_acceleration_scaling_factor(1.0)
    rospy.loginfo("Set Init Pose")
    jetcobot.set_named_target("init")
    jetcobot.go()
    p = PoseStamped()
    p.header.frame_id = end_effector_link
    p.pose.position.x = 0.1
    p.pose.position.y = 0
    p.pose.position.z = 0.01
    p.pose.orientation.w = 1
    # 添加tool
    scene.attach_box(end_effector_link, 'tool', p, [0.035, 0.035, 0.035])
    target_joints1 = [0.3247526227263788, -0.7157521198111748, -0.32114875238797164, 1.0372511142118903, -0.32451764724349896, -0.0010940082355846373]
    target_joints2 = [0.4426930220547772, -2.223432135882768, 0.3570042240467469, 1.8672042535085769, -0.4403060538701685, -0.0013658273509663124]
    table_list = {
        "obj0": [[0.15, 0.02, 0.8], [0.38, -0.2, 0.2]],
        "obj1": [[0.15, 0.02, 0.8], [0.38, 0.2, 0.2]],
        "obj2": [[0.15, 0.42, 0.01], [0.38, 0, 0.6]],
        "obj3": [[0.15, 0.42, 0.01], [0.38, 0, 0.32]],
        "obj4": [[0.15, 0.42, 0.01], [0.38, 0, 0.1]],
    }
    # 添加obj
    for i in range(len(table_list)):
        add_obj(p, list(table_list.keys())[i], table_list[list(table_list.keys())[i]][0],
                table_list[list(table_list.keys())[i]][1])
    rospy.loginfo("Grip Target")
    i = 0
    while i < 5:
        jetcobot.set_joint_value_target(target_joints1)
        jetcobot.go()
        jetcobot.set_joint_value_target(target_joints2)
        jetcobot.go()
        i += 1
        print ("第 {} 次规划!!!".format(i))
        sleep(0.5)
    # 规划完成后复位
    jetcobot.set_named_target("init")
    jetcobot.go()
    scene.remove_attached_object(end_effector_link, 'tool')
    scene.remove_world_object()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
