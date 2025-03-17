# This code is taken from  https://bitw1z.medium.com/bridging-dimensions-camera-calibration-for-2d-to-3d-mapping-3d2b0a060a6f

import cv2 
import numpy as np 

def mouse_callback(event, x, y, flags, param):
    print(f"mouse event, x: {x}, y: {y}")
    
cap = cv2.VideoCapture(0)

# Set resolution explicitly
ret, frame = cap.read()

cv2.namedWindow('frame')
cv2.setMouseCallback('frame', mouse_callback)

while True:
    cv2.imshow('frame', frame)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    
cv2.destroyAllWindows()