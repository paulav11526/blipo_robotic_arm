# This code is taken from https://bitw1z.medium.com/bridging-dimensions-camera-calibration-for-2d-to-3d-mapping-3d2b0a060a6f
# Parameters were changed to fit the current project

import cv2
import numpy as np

# 2D pixel coordinates
points_2D = np.array([
                        (164, 238),  # Left Bottom
                        (417,240),  # Right Bottom
                        (162, 50),  # Left Top
                        (421, 59),  # Right Top
                      ], dtype="double")
                      
# 3D world coordiantes 
points_3D = np.array([
                      (3.3, 18.3, 0.0),     #Left bottom 
                      (24.3, 18.3, 0.0),    # Right Bottom   
                      (3.3, 3.3, 0.0),      # Left Top
                      (24.3, 3.3, 0.0)      # Right Top
                     ], dtype="double")

# camera intrinsic parameter 
# from camera_info.yaml
cameraMatrix = np.array([
                        (481.1706, 0, 319.82166),
						(0, 476.0982, 234.1129), 
						(0, 0, 1)], dtype="double")

# null values 
dist_coeffs = np.zeros((4,1))

#solvePnp 
print("Starting calibration")
retval, rvec, tvec = cv2.solvePnP(points_3D, points_2D, cameraMatrix, 
                                  dist_coeffs, rvec=None, tvec=None, 
                                  useExtrinsicGuess=None, flags=None)
if retval: 
	rvec, _ = cv2.Rodrigues(rvec) # rotation matrix only
	np.save('cam_rotation1.npy', rvec)
	np.save('cam_translation1.npy', tvec)

print("Calibration ended")
