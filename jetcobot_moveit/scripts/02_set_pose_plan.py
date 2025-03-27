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
    jetcobot.set_goal_position_tolerance(0.001)
    jetcobot.set_goal_orientation_tolerance(0.001)
    # 设置允许目标误差
    jetcobot.set_goal_tolerance(0.001)
    # 设置允许的最大速度和加速度
    jetcobot.set_max_velocity_scaling_factor(1.0)
    jetcobot.set_max_acceleration_scaling_factor(1.0)
    # 设置"init"为初始点
    jetcobot.set_named_target("init")
    jetcobot.go()
    sleep(1)
    # 创建位姿实例
    pos = Pose()
    # 设置具体的位置
    pos.position.x = 0.364157
    pos.position.y = -0.0644749
    pos.position.z = 0.334626
    # 四元素
    pos.orientation.x = 0.013323
    pos.orientation.y = -0.0043291
    pos.orientation.z = -0.00100745
    pos.orientation.w = 0.999851

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
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
