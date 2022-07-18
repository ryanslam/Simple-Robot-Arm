from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import os
import time

# File path to save the calirbation images.
path = "./calibration_images"

camera = picamera.PiCamera()
camera.resolution(480,480)
camera.start_preview()

date_string = time.strftime("%Y-%m-%d-%H:%M")

raw_capture = PiRGBArray(camera, size=(480,480))

time.sleep(2)

for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    img = frame.array
    cv2.imshow("Frame", img)

    key = cv2.waitKey(0)
    
    if key == ord("q"):
        cv2.imwrite(path + date_string, img)
    elif key== 27:
        break

    raw_capture.truncate(0)