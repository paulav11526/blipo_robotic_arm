import cv2 
import numpy as np 
import os

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # Only print when the left mouse button is clicked      
        print(f"Mouse click at x: {x}, y: {y}")

cap = cv2.VideoCapture(0)
    

# Set resolution explicitly (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cv2.namedWindow('frame')
cv2.setMouseCallback('frame', mouse_callback)

while True:
    ret, frame = cap.read()  # Update frame inside the loop
    
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow('frame', frame)
    
    k = cv2.waitKey(1) & 0xFF
    if k == 27:  # Press ESC to exit
        break

cap.release()  # Release the camera
cv2.destroyAllWindows()

