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

# camera intrinsic parameters 
# from rostopic echo /camera/color/camera_info
cameraMatrix = np.array([
                        (454.9405822753906, 0.0, 330.1873779296875),
						(0.0, 454.9405822753906, 238.76426696777344), 
						(0.0, 0.0, 1.0)], dtype="double")
dist_coeffs = np.array([0.05183049291372299, -0.07166079431772232, 0.0, 0.0], dtype="double")

# focal length and center pixels from intrisic parameters 
fx = cameraMatrix[0][0]
fy = cameraMatrix[1][1]
cx = cameraMatrix[0][2]
cy = cameraMatrix[1][2]

print("Starting calibration")
retval, rvec, tvec = cv2.solvePnP(points_3D, points_2D, cameraMatrix, 
                                  dist_coeffs, rvec=None, tvec=None, 
                                  useExtrinsicGuess=None, flags=None)
if retval: 
	rvec, _ = cv2.Rodrigues(rvec) # rotation matrix only
print("Calibration ended")
# Verifying the calibration
projected_points, _ = cv2.projectPoints(points_3D, rvec, tvec, cameraMatrix, dist_coeffs)
#print(projected_points)


# rotation and translation matrix from world to camera coordinate system
R_cw = rvec # world to camera
t_cw = tvec# world to camera

# given pixel values, convert it to real world coordinate system 

def pixel_to_world(u, v, cameraMatrix, distCoeffs, R_cw, t_cw, Z_c):
    
    R_wc = R_cw.T # camera to world
    # Translation to world frame
    t_wc = -np.dot(R_wc, t_cw) # camera to world
    
    # Convert pixel (u,v) to normalized camera coordinates
    pixel = np.array([[[u, v]]], dtype=np.float32)
    normalized_pixel = cv2.undistortPoints(pixel, cameraMatrix, distCoeffs)

    # Scale by depth Z_c
    P_c = np.array([[normalized_pixel[0][0][0] * Z_c, normalized_pixel[0][0][1] * Z_c, Z_c]], dtype=np.float32).T


    # Apply world transformation
    P_w = np.dot(R_wc, P_c) + t_wc

    return P_w[:2]  # Return only x, y


# Pixel 192, 328
value = pixel_to_world(192, 328, cameraMatrix, dist_coeffs, R_cw, t_cw, 0.383)
print(value)

