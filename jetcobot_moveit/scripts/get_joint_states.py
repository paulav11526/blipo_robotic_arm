#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import JointState

class JointInfoCollector:
    def __init__(self):
        rospy.init_node("joint_info_collector", anonymous=True)

        # TF listener to get transformations
        self.listener = tf.TransformListener()

        # Joint state subscriber
        self.joint_states = {}
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        # Base Link Frame
        self.base_link = "base_link"

        # List of joint names 
        self.joint_names = [
            "1_Joint", "2_Joint", "3_Joint", "4_Joint", "5_Joint", "6_Joint"
        ]
        # List of link names from frames.pdf
        self.link_names = [
            "1_Link", "2_Link", "3_Link", "4_Link", "5_Link", "6_Link"
        ]

        rospy.sleep(2)  # Give some time for TF to populate

    def joint_state_callback(self, msg):
        # Callback to store the latest joint angles
        rospy.loginfo(f"Received Joint States: {msg.name} -> {msg.position}")
        for i, name in enumerate(msg.name):
            self.joint_states[name] = msg.position[i]

    def get_joint_positions(self):
        # Get all joint positions relative to the end-effector
        joint_positions = {}
        for joint in self.link_names:
            try:
                self.listener.waitForTransform(self.base_link, joint, rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform(self.base_link, joint, rospy.Time(0))
                joint_positions[joint] = {"position": trans, "rotation": rot}
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn(f"Could not find transform for {joint}")

        return joint_positions

    def get_joint_angles(self):
        # Get joint angles from stored joint states
        return {joint: self.joint_states.get(joint, None) for joint in self.joint_names}

    def run(self):
        rospy.loginfo("Collecting joint information...")
        while not rospy.is_shutdown():
            positions = self.get_joint_positions()
            angles = self.get_joint_angles()

            rospy.loginfo("Joint Positions (relative to Base Frame):")
            for joint, data in positions.items():
                rospy.loginfo(f"{joint}: Position {data['position']}, Rotation {data['rotation']}")

            rospy.loginfo("Joint Angles:")
            for joint, angle in angles.items():
                rospy.loginfo(f"{joint}: {angle} rad")

            rospy.sleep(5)

if __name__ == "__main__":
    collector = JointInfoCollector()
    collector.run()