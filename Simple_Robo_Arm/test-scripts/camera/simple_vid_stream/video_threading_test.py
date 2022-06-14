from __future__ import annotations
import sys
sys.path.append(sys.path[0] + '/../../..')
import cv2
import argparse
import os
from threading import Thread, local
from datetime import datetime
import aiohttp
import socketio         # new versions threw errors, worked with 4.3.1
import asyncio
import logging
import time
from typing import Optional

import rsl.rr_two_link.camera as camera
import rsl.rr_two_link.util as util
from rsl.rr_two_link.server import *


# overwrite the index file in server
async def index(request):
    path_to_this_file = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(path_to_this_file, '/index.html')
    with open(filename) as file_obj:
        return aiohttp.web.Response(text = file_obj.read(), content_type='text/html')


async def video_feed(request):
    response = aiohttp.web.StreamResponse()
    response.content_type = 'multipart/x-mixed-replace; boundary=frame'
    await response.prepare(request)

    # get the encoded video feed as bytes and write to cients
    try:
        async for encoded_frame in camera.generate_encoded_video():
            await response.write(encoded_frame)
    except ConnectionResetError:    # the client has disconnected
        pass
    return response


#start the server
if __name__ == '__main__':
    #bind the aiohttp endpoint to the web_application
    web_app.router.add_get('/', index)
    web_app.router.add_get('/video_feed', video_feed)
    import argparse
    parser = argparse.ArgumentParser()
    # Add an argument
    parser.add_argument('--log_level', type=str, required=False)
    # Parse the argument
    args = parser.parse_args()
    log_level_arg:str = args.log_level.upper()
    
    if log_level_arg is None:
        log_level = logging.INFO
    else:
        if log_level_arg == 'DEBUG':
            log_level = logging.DEBUG
        elif log_level_arg == 'INFO':
            log_level = logging.INFO
        elif log_level_arg == 'WARNING':
            log_level = logging.WARNING
        elif log_level_arg == 'ERROR':
            log_level = logging.ERROR
    
    # set up the robot
    server_logger.level = log_level
    try:
        aiohttp.web.run_app(web_app)
    finally:
        server_logger.info(f'Shutting down.')
        print(loop)
        try:
            loop = asyncio.get_running_loop()
        except:
            loop = asyncio.get_event_loop()
        loop.run_until_complete(shut_down(web_app))
        time.sleep(0.5)
        loop.close()











# https://nrsyed.com/2018/07/05/multithreading-with-opencv-python-to-improve-video-processing-performance/

class CountsPerSec:
    """
    Class that tracks the number of occurrences ("counts") of an
    arbitrary event and returns the frequency in occurrences
    (counts) per second. The caller must increment the count.
    """

    def __init__(self):
        self._start_time = None
        self._num_occurrences = 0

    def start(self):
        self._start_time = datetime.now()
        return self

    def increment(self):
        self._num_occurrences += 1

    def countsPerSec(self):
        elapsed_time = (datetime.now() - self._start_time).total_seconds()
        return self._num_occurrences / elapsed_time

# performance without multithreading

class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """

    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):    
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True


class VideoShow:
    """
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False

    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            cv2.imshow("Video", self.frame)
            if cv2.waitKey(1) == ord("q"):
                self.stopped = True

    def stop(self):
        self.stopped = True


def putIterationsPerSec(frame, iterations_per_sec):
    """
    Add iterations per second text to lower-left corner of a frame.
    """

    cv2.putText(frame, "{:.0f} iterations/sec".format(iterations_per_sec),
        (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
    return frame


def noThreading(source=0):
    """Grab and show video frames without multithreading."""

    cap = cv2.VideoCapture(source)
    cps = CountsPerSec().start()

    while True:
        grabbed, frame = cap.read()
        if not grabbed or cv2.waitKey(1) == ord("q"):
            break

        frame = putIterationsPerSec(frame, cps.countsPerSec())
        cv2.imshow("Video", frame)
        cps.increment()


def threadVideoGet(source=0):
    """
    Dedicated thread for grabbing video frames with VideoGet object.
    Main thread shows video frames.
    """

    video_getter = VideoGet(source).start()
    cps = CountsPerSec().start()

    while True:
        if (cv2.waitKey(1) == ord("q")) or video_getter.stopped:
            video_getter.stop()
            break

        frame = video_getter.frame
        frame = putIterationsPerSec(frame, cps.countsPerSec())
        cv2.imshow("Video", frame)
        cps.increment()


def threadVideoShow(source=0):
    """
    Dedicated thread for showing video frames with VideoShow object.
    Main thread grabs video frames.
    """

    cap = cv2.VideoCapture(source)
    (grabbed, frame) = cap.read()
    video_shower = VideoShow(frame).start()
    cps = CountsPerSec().start()

    while True:
        (grabbed, frame) = cap.read()
        if not grabbed or video_shower.stopped:
            video_shower.stop()
            break

        frame = putIterationsPerSec(frame, cps.countsPerSec())
        video_shower.frame = frame
        cps.increment()


def threadBoth(source=0):
    """
    Dedicated thread for grabbing video frames with VideoGet object.
    Dedicated thread for showing video frames with VideoShow object.
    Main thread serves only to pass frames between VideoGet and
    VideoShow objects/threads.
    """

    video_getter = VideoGet(source).start()
    video_shower = VideoShow(video_getter.frame).start()
    cps = CountsPerSec().start()

    while True:
        if video_getter.stopped or video_shower.stopped:
            video_shower.stop()
            video_getter.stop()
            break

        frame = video_getter.frame
        frame = putIterationsPerSec(frame, cps.countsPerSec())
        video_shower.frame = frame
        cps.increment()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", "-s", default=0,
        help="Path to video file or integer representing webcam index"
            + " (default 0).")
    ap.add_argument("--thread", "-t", default="none",
        help="Threading mode: get (video read in its own thread),"
            + " show (video show in its own thread), both"
            + " (video read and video show in their own threads),"
            + " none (default--no multithreading)")
    args = vars(ap.parse_args())

    # If source is a string consisting only of integers, check that it doesn't
    # refer to a file. If it doesn't, assume it's an integer camera ID and
    # convert to int.
    if (
        isinstance(args["source"], str)
        and args["source"].isdigit()
        and not os.path.isfile(args["source"])
    ):
        args["source"] = int(args["source"])

    if args["thread"] == "both":
        threadBoth(args["source"])
    elif args["thread"] == "get":
        threadVideoGet(args["source"])
    elif args["thread"] == "show":
        threadVideoShow(args["source"])
    else:
        noThreading(args["source"])


# if __name__ == "__main__":
#     main()