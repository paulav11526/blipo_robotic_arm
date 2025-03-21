import numpy as np

# Replace with the path to your .npy file
file_path = '/home/jetson/jetcobot_ws/src/blipo_robotic_arm/astra_tracker/scripts/cam_translation2.npy'

# Load the .npy file
data = np.load(file_path)

# Print the content of the file
print(data)

