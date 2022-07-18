import cv2
import os
import picamera
import time

# File path to save the calirbation images.
path = "./calibration_images"

camera = picamera.PiCamera()
camera.resolution(480,480)
camera.start_preview()

date_string = time.strftime("%Y-%m-%d-%H:%M")

sleep(5)
camera.capture(path + date_string)