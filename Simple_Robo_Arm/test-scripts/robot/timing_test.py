from __future__ import annotations
import sys
import logging
import math
import time
import statistics
import numpy as np

sys.path.append(sys.path[0] + '/../..')
from rsl.rr_two_link.robotics.robot import *


arm = RRTwoLink(log_level=logging.WARNING)


def timing_test(func, args=[]):
    times = []
    for _ in range(10):
        start = time.monotonic()
        func(*args)
        end = time.monotonic()
        t = end - start
        times.append(t)
        time.sleep(0.1)
    print(f'Mean time for execute function {func.__name__}: {round(statistics.mean(times), 4)}, variance: {statistics.variance(times)}')


timing_test(arm.torque_enable)
timing_test(arm.set_joint_pose, [[0, np.pi]])       # result: about 0.0286

timing_test(arm.read_motor_states)    # result: about 0.0605



