import cv2
import numpy as np
from calibrate import calibrate
from detect_blob import detect_blob
from localise import localise
from import_mask_params import import_mask_params

# calibrate()            # Uncomment if you want to run the calibration program for mask parameters

frame = cv2.imread("test.jpg", 1)
mask_params= import_mask_params()
LED_list= detect_blob(frame, mask_params)
robot_list= localise(LED_list)
print(robot_list)
