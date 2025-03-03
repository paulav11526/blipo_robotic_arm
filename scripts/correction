#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import Float32MultiArray
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot

# This file computes the updated joint angles based on the PID controller output, then sends the updated joint angles to the robot


class Correction:
    def __init__(self):
        # Initialize the joint angles
        self.joint_angles = [0, 0, 0, 0, 0, 0]
        
        rospy.Subscriber("/control", Float32MultiArray, self.callback)
        rospy.loginfo("Correction node started, waiting for messages ...")

        #port = rospy.get_param("~port", str(os.getenv('MY_SERIAL')))
        #baud = rospy.get_param("~baud", 1000000)
    
        #self.mc = MyCobot(port, baud)        
        

    def callback(self, data):
        # data is the control signal from PID controller
        # Update the joint angles based on the PID controller output
        rospy.loginfo(f"Received Control Signal: {data.data}")
        self.joint_angles = self.update_joint_angles(data.data)
        # Send the updated joint angles to the robot
        rospy.loginfo(f"Simulated send: {self.joint_angles}")
        #self.mc.send_radians(self.joint_angles, 80)
        

        


    def update_joint_angles(self, controller_output):
        # Call the IK solver to get the updated joint angles
        rospy.loginfo(f"Controller_output: {controller_output}")

        ik_service = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.wait_for_service("/compute_ik")

        request = GetPositionIKRequest()
        request.ik_request.group_name = "arm_group"
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = JointState()
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose.position.x = controller_output[0]
        request.ik_request.pose_stamped.pose.position.y = controller_output[1]
        request.ik_request.pose_stamped.pose.position.z = controller_output[2]
        request.ik_request.pose_stamped.pose.orientation.x = 0
        request.ik_request.pose_stamped.pose.orientation.y = 0
        request.ik_request.pose_stamped.pose.orientation.z = 0
        request.ik_request.pose_stamped.pose.orientation.w = 1

        try: 
            response = ik_service(request)
            rospy.loginfo(f"IK Solver Response: {response}")
            if response.error_code.val == response.error_code.SUCCESS:
                self.joint_angles = response.solution.joint_state.position
                rospy.loginfo(f"Updated Joint Angles: {self.joint_angles}")
                return self.joint_angles
            else:
                rospy.logerr("IK computation failed")
                return None
                  
        except:
            rospy.logerr("Service call failed")
            return None

if __name__ == "__main__":
    rospy.init_node('correction_subscriber', anonymous=True)
    rospy.loginfo("Correction node initialized")
    correction = Correction()
    rospy.spin()  # Keep the node alive
