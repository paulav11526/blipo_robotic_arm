#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point 

class CameraCorrection:
    def __init__(self):
        rospy.init_node("camera_correction", anonymous=True)
        rospy.Subscriber("KCF_Tracker/coordinates", Point, self.depth_correction)       

    def depth_correction(self, msg):
        rospy.loginfo(f'Tracker coordinates: {msg}')
        z_actual = (msg.z * 0.8090) - 0.0342
        rospy.loginfo(f'Corrected depth: {z_actual}')


    def run(self):
        rospy.loginfo("Camera_correction node running...")
        rospy.spin()
    
 
if __name__ == '__main__':
    corrector = CameraCorrection()
    corrector.run()

