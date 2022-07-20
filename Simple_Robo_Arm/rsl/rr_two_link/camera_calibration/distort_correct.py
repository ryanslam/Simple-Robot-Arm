# From: https://github.com/niconielsen32/ComputerVision/blob/master/cameraCalibration.py
import cv2
import numpy as np
import glob

# Initialize constants.
CHESSBOARD_SIZE = (9,6)
CHESSBOARD_SQUARE_SIZE = 25
FRAME_SIZE = (480, 480)

# When we want to terminate the iteration.
terminate = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:CHESSBOARD_SIZE[0],0:CHESSBOARD_SIZE[1]].T.reshape(-1,2)

objp = objp * CHESSBOARD_SQUARE_SIZE

# Array for obj point(3D) and image point(2D)
obj_points = []
image_points = []

images = glob.glob('./calibration_images/*.jpg')

for image in images:
    img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the corners of the chess board.
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
    
    # Add the found corners to the obj and image array.
    if ret == True:
        obj_points.append(objp)
        sub_corner = cv2.cornerSubPix(gray, corners, (6,6), (-1,-1), terminate)
        image_points.append(corners)

        # Draw the corners.
        # cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, sub_corner, ret)
        # cv2.imshow('img', img)
        # cv2.waitKey(1000)

# Calibrate the camera.
ret, cam_matrix, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, image_points, FRAME_SIZE, None, None)

# Undistortion section.
img = cv2.imread('./calibration_images/2022-07-18-15:01:39.jpg')
h, w = img.shape[:2]
new_cam_matrix, roi = cv2.getOptimalNewCameraMatrix(cam_matrix, dist, (w,h), 1, (w,h))

# Undistort the image.
dst = cv2.undistort(img, cam_matrix, dist, None, new_cam_matrix)
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('caliResult1.jpg', dst)
