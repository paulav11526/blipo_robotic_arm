#!/usr/bin/env python
# coding: utf-8
import rospy
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler

# 角度转弧度
DE2RA = pi / 180

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node("set_pose_py")
    # 初始化机械臂
    jetcobot = MoveGroupCommander("arm_group")
    # 当运动规划失败后，允许重新规划
    jetcobot.allow_replanning(True)
    jetcobot.set_planning_time(5)
    # 尝试规划的次数
    jetcobot.set_num_planning_attempts(10)
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    jetcobot.set_goal_position_tolerance(0.01)
    jetcobot.set_goal_orientation_tolerance(0.01)
    # 设置允许目标误差
    jetcobot.set_goal_tolerance(0.01)
    # 设置允许的最大速度和加速度
    jetcobot.set_max_velocity_scaling_factor(0.1)
    jetcobot.set_max_acceleration_scaling_factor(0.1)
    # 设置"init"为初始点
    jetcobot.set_named_target("init")
    jetcobot.go()
    sleep(1)
    # 创建位姿实例
    pos = Pose()
    
    
    
    # Back
    pos.position.x = -0.226 
    pos.position.y = -0.061
    pos.position.z = 0.3
    # 四元素
    pos.orientation.x = -0.008
    pos.orientation.y = -0.591
    pos.orientation.z = -0.004
    pos.orientation.w = 0.807
    # 设置目标点
    jetcobot.set_pose_target(pos)
    plan = jetcobot.plan()
    # print("plan = ",plan)
    if plan[0]==True:
        rospy.loginfo("plan success")
        # 规划成功后运行
        jetcobot.execute(plan[1])
    else:
        rospy.loginfo("plan error")
        
    # Right
    pos.position.x = 0.114
    pos.position.y = -0.178
    pos.position.z = 0.1
    # 四元素
    pos.orientation.x = 5.990905825550542e-05
    pos.orientation.y = 3.45385976533631e-05
    pos.orientation.z = -2.9922657689618833e-05
    pos.orientation.w = 0.9999999971613122
    # 设置目标点
    jetcobot.set_pose_target(pos)
    plan = jetcobot.plan()
    # print("plan = ",plan)
    if plan[0]==True:
        rospy.loginfo("plan success")
        # 规划成功后运行
        jetcobot.execute(plan[1])
    else:
        rospy.loginfo("plan error")
    sleep(1)
    
   
    # Crunched
    pos.position.x = 0.094
    pos.position.y = -0.063
    pos.position.z = 0.242
    # 四元素
    pos.orientation.x = 0.000
    pos.orientation.y = 0.044
    pos.orientation.z = -0.000
    pos.orientation.w = 0.999
    # 设置目标点
    jetcobot.set_pose_target(pos)
    plan = jetcobot.plan()
    # print("plan = ",plan)
    if plan[0]==True:
        rospy.loginfo("plan success")
        # 规划成功后运行
        jetcobot.execute(plan[1])
    else:
        rospy.loginfo("plan error")
    #moveit_commander.roscpp_shutdown()
    #moveit_commander.os._exit(0)
    sleep(1)
        
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    sleep(1)
