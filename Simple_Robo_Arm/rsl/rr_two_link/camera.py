#!/usr/bin/env python
"""
@author Bruce Iverson
"""

# https://www.pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/
# https://pyshine.com/Online-Video-Processing-From-Client-Camera/
from __future__ import annotations
import picamera
from picamera import encoders
from picamera.array import PiRGBArray
import time
import numpy as np
import cv2
import pickle
from logging import DEBUG, INFO, WARNING
import asyncio
from typing import Optional
from datetime import datetime
import traceback
import time
from threading import Thread

try:
    import util
except:
    import rsl.rr_two_link.util as util


#####################################################################
########################## Machine vision ###########################
#####################################################################
def perform_all_machine_vision(frame:np.ndarray, text_overlay:Optional[list]) -> np.ndarray:
    """This function encapsulate all of the machine vision processes including annotation"""
    return annotate_frame(frame)


def correct_fisheye(frame:np.ndarray) -> np.ndarray:
    """Correct for the fisheye lens using cv2"""
    pass


def annotate_frame(frame:np.ndarray,text_overlay:Optional[list]=None) -> np.ndarray:
    now = datetime.now()
    text = datetime.strftime(now, "%m/%d/%Y, %H:%M:%S")
    resolution = frame.shape
    
    scale = 0.5
    color = (255, 255, 255)
    origin = (5, 20)
    annotated = cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
            scale, color)

    if text_overlay:
        for item in text_overlay:
            origin = (origin[0], origin[1] + 25)
            annotated = cv2.putText(frame, item, origin, cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color)
    return annotated
    

#####################################################################
########################## CAMERA HANDLERS ##########################
#####################################################################
class FPSCounter:
    def __init__(self, k:float=.1):
        self.fps = 0
        self._smoothing_constant = k
        self._last_update_time = None
        self._n_frames = 0

    def start(self):
        self._last_update_time = time.monotonic()
        self.fps = 0
        self._n_frames = 0

    def update(self):
        """This should be called every frame (for video)"""
        self._n_frames += 1
        now = time.monotonic()
        dt = now - self._last_update_time
        self._last_update_time = now
        fps_for_this_frame = 1/dt
        self.fps = (fps_for_this_frame - self.fps) * self._smoothing_constant + self.fps

    def get_fps(self) -> float:
        return round(self.fps, 1)


class CameraWrapper:
    """A wrapper for the camera, supporting images and video streams including encoded video.
    This class uses a separate thread to handle the I/O of the camera in order to increase framerate.

    Partially inspired by: https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/
    """
    
    def __init__(self, resolution=(480, 480), framerate=32):
        self.logger = util.get_simple_logger('camera', verbosity=DEBUG)

        self.logger.debug(f'Initializing camera with res: {resolution}, framerate: {framerate}')
        self.camera:Optional[picamera.PiCamera] = picamera.PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate        # in standalone tests this seemed to be capped at 10 fps,
        # although I was getting low voltage warnings at the time
        time.sleep(2)
        # camera.vflip = False
        # camera.contrast = 10
        self.rawCapture = PiRGBArray(self.camera, size=self.camera.resolution)

        # video helpers
        self._current_frame = None
        self._raw_stream = None
        self._fps_estimator = FPSCounter()
        self._stop_video_flag = True
        # unused for now. may just want to stuff it in

    def take_and_display_image(self):
        self.logger.info(f'Taking a picture!')
        # grab an image from the camera
        self.camera.capture(self.rawCapture, format="bgr")
        image = self.rawCapture.array    # numpy array shape 480, 720, 3
        return image

    def start_video(self):
        """start the thread to read frames from the video stream"""
        self.logger.info(f'Begin raw video capture')
        self._stop_video_flag = False
        self._raw_stream = self.camera.capture_continuous(
            self.rawCapture,
            format='bgr',
            use_video_port=True
            )
        Thread(target=self._capture_video, args=()).start()
    
    def _capture_video(self):
        """keep looping infinitely until the thread is stopped"""
        self._fps_estimator.start()
        for frame in self._raw_stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self._current_frame = frame.array
            self.rawCapture.truncate(0)
            self._fps_estimator.update()
            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self._stop_video_flag:
                self.logger.info(f'Ending raw capture FPS estimation: {self._fps_estimator.get_fps()}')
                self._current_frame = None
                return

    def get_latest_frame(self):
        # return the frame most recently read
        fps_text = f'Measured FPS: {self._fps_estimator.get_fps()}'
        if self._current_frame is None:
            return None
        return perform_all_machine_vision(self._current_frame, fps_text)
        
    def get_encoded_frame(self):
        if self._current_frame is None:
            self.logger.warning(f'Frame is None, camera likely not running')
            return None
        # encode the image
        # (success, encoded_image) = cv2.imencode(".jpg", self._current_frame)  # note this could be [1].tobytes()
        (success, encoded_image) = cv2.imencode(".jpg", self.get_latest_frame())  # note this could be [1].tobytes()
        if not success:
            self.logger.warning(f'Image encoding failed.')
            return None
        return (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + encoded_image.tobytes() + b'\r\n')

    async def encoded_video_generator(self):
        # hacky approximation of fps cause its in another thread...
        if self._stop_video_flag:
            self.logger.error(f'Video is not running, cannot get encoded stream')
        fps = FPSCounter(k=.15)     # takes an average fps for debugging/performance evaluation
        fps.start()
        await asyncio.sleep(float(1/self.camera.framerate))
        # await asyncio.sleep(float(1/self.camera.framerate))
        while 1:
            fps.update()
            if self._stop_video_flag:
                self.logger.info(f'Encoded video ending. fps: {fps.get_fps()}')
                return
            
            sleep_time = 1/self.camera.framerate - 0.005
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            yield self.get_encoded_frame()

    def stop_video(self):
        # indicate that the thread should be stopped
        self._stop_video_flag = True

    def clean_up(self):
        self.stop_video()
        time.sleep(0.2)
        try:
            self._raw_stream.close()
        except ValueError:
            pass
        finally:
            self._raw_stream = None
        self.rawCapture.close()
        self.camera.close()
        return
    
# def generate_video_old(size=(480,480), framerate=32) -> np.ndarray:
#     """An unoptimized video generator"""
#     logger.info(f'Begin generate_video')
#     try:
#         raw_capture = PiRGBArray(self.camera, size=self.camera.resolution)
#         # capture frames from the camera
#         counter = FPSCounter()
#         counter.start()
#         for frame in self.camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
#             image = frame.array
#             counter.update()
#             fps_text = f'Measured FPS: {round(counter.fps,2)}'
#             logger.debug(fps_text)
#             image = perform_all_machine_vision(image, fps_text)
#             # clear the stream in preparation for the next frame
#             raw_capture.truncate(0)
#             yield image

#         logger.info(f'FPS measured as: {fps_text}')

#     except Exception as e:
#         logger.error(f'An exception was encounted.')
#         logger.error(traceback.print_exc())
#     finally:
#         logger.info(f'Video generation ended.')
#         self.camera.close()

# def generate_encoded_video_old() -> bytes:
#     for frame in self.generate_video_old():
#         if frame is None:
#             logger.warning(f'Frame is None, camera likely not running')
#             continue
#         # encode the image
#         (success, encoded_image) = cv2.imencode(".jpg", frame)  # note this could be [1].tobytes()
#         if not success:
#             logger.warning(f'Image encoding failed.')
#             continue
#         # yield the output frame in the byte format
#         yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + encoded_image.tobytes() + b'\r\n')
