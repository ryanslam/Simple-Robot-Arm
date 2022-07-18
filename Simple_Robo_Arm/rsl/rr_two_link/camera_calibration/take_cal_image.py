from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import os
import time

# File path to save the calirbation images.
path = "./calibration_images/"

# Initialize the pi camera module with a resolution (480,480)
camera = PiCamera()
camera.resolution = (480,480)
camera.start_preview()

# Initialize the capture array.
raw_capture = PiRGBArray(camera, size=(480,480))

time.sleep(2)           # Camera bootup time.

# Continuously capture the camera frames.
for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    img = frame.array
    # Display the image using cv2
    cv2.imshow("Frame", img)

    key = cv2.waitKey(0)
    
    # Store frame as calibration image.
    if key == ord("q"):
        date_string = time.strftime("%Y-%m-%d-%H:%M:%S")
        cv2.imwrite(path + date_string +'.jpg', img)
    # Exit from the capturing loop.
    elif key== 27:
        break
    
    # Traverse to the next frame.
    raw_capture.truncate(0)