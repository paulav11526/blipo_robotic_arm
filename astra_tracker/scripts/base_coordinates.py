#!/usr/bin/env python

import rospy
import cv2 
import numpy as np 
import tf
import math
from geometry_msgs.msg import Point
from transforms3d.quaternions import quat2mat
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PointStamped


class PixelToBase:
    def __init__(self):
        rospy.init_node("camera_transformation", anonymous=True)

        self.tf_listener = tf.TransformListener()
        self.base_pub = rospy.Publisher("base_coordinates", Point, queue_size=10)
        rospy.Subscriber("KCF_Tracker/coordinates", Point, self.pixel_callback)
        self.camera_pub = rospy.Publisher('/camera_point', PointStamped, queue_size=10)
        self.latest_pixel = None

        # Camera intrinsic parameters
        self.cameraMatrix = np.array([
                            (454.9405822753906, 0.0, 330.1873779296875),
                            (0.0, 454.9405822753906, 238.76426696777344), 
                            (0.0, 0.0, 1.0)], dtype="double")
        
        self.dist_coeffs = np.array([0.05183049291372299, -0.07166079431772232, 0.0, 0.0], dtype="double")
   
    def get_transform(self):
        
        try:
            self.tf_listener.waitForTransform('base_link', 'camera_link', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('base_link', 'camera_link', rospy.Time(0))
                      
            # Convert quaternion to rotation matrix
            r = R.from_quat(rot)
            rotation_matrix = r.as_matrix()
            
            # Convert translation and rotation to transformation matrix
            T = np.identity(4)
            T[:3, :3] = rotation_matrix
            T[:3, 3] = trans[:3]    
            rospy.loginfo(f"T between base link and camera_link: {T}")

            # Return the transformation matrix between base frame and camera
            return T    

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"Could not find transform: {e}")
                return None
        
        
    def pixel_callback(self, msg):
        if math.isnan(msg.z) or msg.z == 0 or math.isinf(msg.z):
            rospy.logwarn("Depth is 0, skipping transformation")
            return
            
        else:
            #z_actual = (msg.z * 0.8090) - 0.0342 # Depth correction (calibration 1)
            #z_actual = (msg.z * 0.8016) - 0.0041 # Depth correction (calibration 2)
            #z_actual = (np.square(msg.z) * 0.0235) + (msg.z * 0.7776) + 0.0017 # Depth correction (calibration 3)
            z_actual = (np.square(msg.z) * -0.1816) + (msg.z * 1.0148) - 0.0434 # Depth correction (calibration 4)
            self.latest_pixel = (msg.x, msg.y, z_actual)  # Store pixel coordinates
            rospy.loginfo(f'z_actual with #4: {z_actual}')\

       
    def pixel_to_base_transform(self, u, v, Z_c, T_bc):

        # Make a pixel array
        pixel = np.array([[[u, v]]], dtype=np.float32)

        # Normalize the pixel
        normalized_pixel = cv2.undistortPoints(pixel, self.cameraMatrix, self.dist_coeffs)

        # Scale by depth Z_c
        P_c = np.array([[normalized_pixel[0][0][0] * Z_c, normalized_pixel[0][0][1] * Z_c, Z_c]], dtype=np.float32).T

        # Homogeneous coordinates
        P_c = np.vstack((P_c, np.ones((1, P_c.shape[1]))))
        rospy.loginfo(f"camera coordinate: {P_c}")

        # Transformation matrix: camera frame wrt base frame   
        P_b = np.dot(T_bc, P_c )
        rospy.loginfo(f"P wrt base frame: {P_b}")

        point_msg = PointStamped()
        point_msg.header.frame_id = "camera_link"
        point_msg.header.stamp = rospy.Time.now()
        point_msg.point.x = P_c[0]
        point_msg.point.y = P_c[1]
        point_msg.point.z = P_c[2]
        self.camera_pub.publish(point_msg)       

        return P_b[:3]  # Return only x, y, z
    
    def run(self):
        rospy.loginfo("Collecting pixel information...")
        rate = rospy.Rate(2)  
        
        while not rospy.is_shutdown():
            if self.latest_pixel is not None:  # Check if pixel data is available
                u, v, z_c = self.latest_pixel                  
                rospy.loginfo(f"Pixel Coordinates: {u}, {v}, {z_c}")
                    
                T_b6 = self.get_transform()
                if T_b6 is None:
                    rospy.logwarn("Transform not available, skipping transformation")
                    rate.sleep()
            
                else:  # Ensure transform was successful
                    base_coordinates = self.pixel_to_base_transform(u, v, z_c, T_b6)
                    rospy.loginfo(f'Base Coordinates: {base_coordinates}')
                    self.base_pub.publish(Point(base_coordinates[0], base_coordinates[1], base_coordinates[2]))
                
            rate.sleep()


if __name__ == "__main__":
    transformer = PixelToBase()
    transformer.run()
                        




