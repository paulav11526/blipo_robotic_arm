#!/usr/bin/env python
# coding: utf-8
import rospy
from math import pi
from time import sleep
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler

# 角度转弧度
DE2RA = pi / 180

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node("set_joint_py", anonymous=True)
    # 初始化机械臂
    jetcobot = MoveGroupCommander("arm_group")
    # 当运动规划失败后，允许重新规划
    jetcobot.allow_replanning(True)
    jetcobot.set_planning_time(5)
    # 尝试规划的次数
    jetcobot.set_num_planning_attempts(10)
    # 设置允许目标角度误差
    jetcobot.set_goal_joint_tolerance(0.001)
    # 设置允许的最大速度和加速度
    jetcobot.set_max_velocity_scaling_factor(1.0)
    jetcobot.set_max_acceleration_scaling_factor(1.0)
    # 设置"init"为初始点
    jetcobot.set_named_target("init")
    jetcobot.go()
    sleep(0.5)
    # 设置目标点 弧度
    joints = [1.9765225621080145, -1.2912211225228438, 1.8984548883544226, -1.1857277805913173, -0.6667299450578678, 0.22747361348948375]
    jetcobot.set_joint_value_target(joints)
    # 多次执行,提高成功率
    for i in range(5):
        # 运动规划
        plan = jetcobot.plan()
        # print("plan = ",plan)
        if plan[0]==True:
            rospy.loginfo("plan success")
            # 规划成功后运行
            jetcobot.execute(plan[1])
            break
        else:
            rospy.loginfo("plan error")
