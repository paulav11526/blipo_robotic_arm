# This code is taken from https://bitw1z.medium.com/bridging-dimensions-camera-calibration-for-2d-to-3d-mapping-3d2b0a060a6f
# Parameters were changed to fit the current project

import cv2
import numpy as np

# 2D pixel coordinates
points_2D = np.array([
                        (192, 328),  # Left Bottom
                        (436, 320),  # Right Bottom
                        (186, 152),  # Left Top
                        (432, 146),  # Right Top
                      ], dtype="double")
                      
# 3D world coordiantes 
points_3D = np.array([
                      (-0.105, -0.075, 0.0),     #Left bottom 
                      (0.105, -0.075, 0.0),    # Right Bottom   
                      (-0.105, 0.075, 0.0),      # Left Top
                      (0.105, 0.075, 0.0)      # Right Top
                     ], dtype="double")

# camera intrinsic parameter 
# from rostopic echo /camera/color/camera_info
cameraMatrix = np.array([
                        (454.9405822753906, 0.0, 330.1873779296875),
						(0.0, 454.9405822753906, 238.76426696777344), 
						(0.0, 0.0, 1.0)], dtype="double")
#print(cameraMatrix)

# null values 
dist_coeffs = np.array([0.05183049291372299, -0.07166079431772232, 0.0, 0.0], dtype="double")
print(dist_coeffs)
#solvePnp 
print("Starting calibration")
retval, rvec, tvec = cv2.solvePnP(points_3D, points_2D, cameraMatrix, 
                                  dist_coeffs, rvec=None, tvec=None, 
                                  useExtrinsicGuess=None, flags=None)
if retval: 
	rvec, _ = cv2.Rodrigues(rvec) # rotation matrix only
	np.save('cam_rot1.npy', rvec)
	np.save('cam_trans1.npy', tvec)

projected_points, _ = cv2.projectPoints(points_3D, rvec, tvec, cameraMatrix, dist_coeffs)
print(projected_points)

print("rvec: \n", rvec, "tvec: \n",tvec)

print("Calibration ended")
