import cv2 
import numpy as np 

cameraMatrix = np.array([
                        (481.17026, 0.0, 319.82166),
						(0.0, 476.09892, 234.1129), 
						(0.0, 0.0, 1.0)], dtype="double")

# focal length and center pixels from intrisic parameters 
fx = cameraMatrix[0][0]
fy = cameraMatrix[1][1]
cx = cameraMatrix[0][2]
cy = cameraMatrix[1][2]
print(f'fx: {fx}, fy: {fy}, cx: {cx}, cy:{cy}')

# rotation and translation matrix from world to camera coordinate system
rot_path = '/home/jetson/jetcobot_ws/src/blipo_robotic_arm/astra_tracker/scripts/cam_rotation2.npy'
trans_path = '/home/jetson/jetcobot_ws/src/blipo_robotic_arm/astra_tracker/scripts/cam_translation2.npy'
R_cw = np.load(rot_path)
t_cw = np.load(trans_path)

# given pixel values, convert it to real world coordinate system 
def convert_2D_to_3D(u, v):
    x = (u - cx) / fx
    y = (v - cy) / fy
    print(f'x: {x}, y: {y}')
    P_c = np.array([[x, y, 1]]).T
    O_c = np.array([[0, 0, 0]]).T

    R_wc = R_cw.T
    P_w = np.dot(R_wc, (P_c - t_cw))
    O_w = np.dot(R_wc, (O_c - t_cw))

    line = P_w - O_w
    line_z = P_w[2][0]
    O_w_z = O_w[2][0]

    k = (-1)*(O_w_z/line_z) # 0 = O_w_z + k*line_z
    P = O_w + k*(P_w - O_w)
    return P


# Pixel 223, 69
value = convert_2D_to_3D(356, 205)
print(value)