#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot
import time


mc = None
gripper_value = []

def get_timestamp():
    return round(time.time())

def callback(data):
    # print(data.position)
    data_list = []  
    for index, value in enumerate(data.position):
        data_list.append(round(value,3))
    # print(data_list[6:])
    
    data_list = data_list[:7]
    # 纠正joint6数据
    data_list[5] = data_list[5]-0.7853981633974483
    mc.send_radians(data_list[:6], 80)
    gripper_value = int(abs(-0.7-data_list[6])* 117)
    mc.set_gripper_value(gripper_value, 80)
    if get_timestamp() % 10 == 0:  
        print("radians:%s"%data_list[:6])
        print("gripper_value:%s"%gripper_value)

    

def listener():
    global mc
    global gripper_value 
    
    rospy.init_node("mycobot_reciver", anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    port = rospy.get_param("~port", '/dev/ttyUSB0')
    baud = rospy.get_param("~baud", 1000000)
    print(port, baud)
    mc = MyCobot(port, baud)
    # spin() simply keeps python from exiting until this node is stopped
    # spin()只是阻止python退出，直到该节点停止
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
