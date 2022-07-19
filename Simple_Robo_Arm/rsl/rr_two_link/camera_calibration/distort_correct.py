# From: https://github.com/niconielsen32/ComputerVision/blob/master/cameraCalibration.py
import cv2
import numpy as np
import glob

# Initialize constants.
CHESS_BOARD_SIZE = (9,6)
FRAME_SIZE = (480, 480)

# When we want to terminate the iteration.
terminate = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)



images = glob.glob('./calibration_images/*.jpg')