#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class Controller:
    def __init__(self):
        rospy.init_node("joint_info_collector", anonymous=True) #Initialize node

        # TF listener to get transformations
        self.listener = tf.TransformListener() 

        # Base Link Frame
        self.base_link = "base_link"

        self.link_names = [
            "1_Link", "2_Link", "3_Link", "4_Link", "5_Link", "6_Link"
        ]

        self.prev_error = [0.0, 0.0, 0.0]  # Stores the last error for derivative term
        self.error_history = []  # Stores all errors for integral term
        self.integral_limit = 10.0 

        self.pub = rospy.Publisher('/control', Float32MultiArray, queue_size=10)
        rospy.sleep(2)  # Give some time for TF to populate


    def get_joint_positions(self):
        # Get all joint positions relative to the base
        joint_positions = {}
        for joint in self.link_names:
            try:
                self.listener.waitForTransform(self.base_link, joint, rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform(self.base_link, joint, rospy.Time(0))
                joint_positions[joint] = {"position": trans, "rotation": rot}
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn(f"Could not find transform for {joint}")

        return joint_positions

    def get_error(self, ref_pos, joint_positions):
        # Computes the error between the current and reference position

        # Subscribe to reference position 
        # ------------------------------------------------------- to implement later on
        
        # Loop to compare error
        if "6_Link" not in joint_positions:
            rospy.logwarn("End effector position unavailable.")
            return none

        current_position = joint_positions['6_Link']["position"]
        error = [ref_pos[i] - current_position[i] for i in range(3)]
        rospy.loginfo(f"Error: {error}")
            
        # Store error history for integral term
        # Avoid integral wind-up
        if len(self.error_history) > self.integral_limit:
            self.error_history.pop(0)

        self.error_history.append(error)
        self.prev_error = error.copy()  # Update previous error
        return error

    def pid_controller(self, error):
        # Implements a PID controller to control the robot arm
        
        # Define PID parameters
        kp = 5
        ki = 0.1
        kd = 0.1

        # Initializing lists to store proportional, integral and derivative terms
        proportional = [0.0] * 3
        integral = [0.0] * 3
        derivative = [0.0] * 3
        controller_output = [0.0] * 3

        # Loop to calculate PID controller
        for i in range(3):
            proportional[i] = error[i] * kp  # Proportional term
            integral[i] = ki * sum(e[i] for e in self.error_history)  # Integral term
            derivative[i] = kd * (error[i] - self.prev_error[i])  # Derivative
            controller_output[i] = proportional[i] + integral[i] + derivative[i]

        
        rospy.loginfo(f"Controller: {controller_output}")
        return controller_output
        
    def pub_control_signal(self, control_signal):
        # Publish control signal
        msg = Float32MultiArray()
        msg.data = control_signal
        self.pub.publish(msg)



    def run(self):
        # Main loop to continuously compute and publish control signals
        rospy.loginfo("Collecting joint information...")
        rate = rospy.Rate(10) #10
        ref_pos = [0.094, -0.063, 0.242]

        while not rospy.is_shutdown():
            positions = self.get_joint_positions()
            err = self.get_error(ref_pos, positions)
            controller = self.pid_controller(err)
            self.pub_control_signal(controller)
            rate.sleep()
            

if __name__ == "__main__":
    control = Controller()
    control.run()
