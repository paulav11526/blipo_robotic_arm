#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time
import rospy
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot


mc = None

def get_timestamp():
    return round(time.time())

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        # if index != 2:
        #     value *= -1
        data_list.append(value)
    
    # 纠正joint6数据
    data_list[5] = data_list[5]-0.7853981633974483
    mc.send_radians(data_list, 80)


def listener():
    global mc
    rospy.init_node("mycobot_reciver", anonymous=True)

    port = rospy.get_param("~port", '/dev/ttyUSB0')
    baud = rospy.get_param("~baud", 1000000)
    print(port, baud)
    mc = MyCobot(port, baud)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止 python 退出，直到该节点停止
    rospy.spin()


if __name__ == "__main__":
    listener()
