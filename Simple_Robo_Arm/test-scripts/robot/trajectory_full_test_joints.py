#!/usr/bin/env python
"""
@author Bruce Iverson
"""

from __future__ import annotations
import sys
import logging
from math import pi
import time
import numpy as np
import sys
import copy
import asyncio

sys.path.append(sys.path[0] + '/../..')
from rsl.rr_two_link.robotics.robot import *


def run(local_way_points:list[tuple[float,float]]):
    global rr_two_link
    rr_two_link.read_motor_states()
    # pop out the starting waypoint if we are already there
    start = rr_two_link.X
    if all([abs(i - j) < 10 for i, j in zip(start, local_way_points[0])]):		# already near the starting point
        local_way_points.pop(0)

    for point in local_way_points:
        print(f'Beginning trajectory movement')
        rr_two_link.set_joint_pose(point, True)
        while rr_two_link.has_trajectory:
            time.sleep(0.25)


loop = asyncio.get_event_loop()
rr_two_link = RRTwoLink(log_level=logging.DEBUG)
# rr_two_link.start_service(loop)
service_task = loop.create_task(rr_two_link._service())

rr_two_link.torque_enable()
q_start = (0, 0)
way_points = [q_start, (np.pi, np.pi)]

rr_two_link.set_joint_pose(q_start)
time.sleep(1)

run(copy.deepcopy(way_points))
rr_two_link.set_joint_pose(q_start)
time.sleep(1)
rr_two_link.set_control_mode(ControlTypes.JOINT_PID)
run(copy.deepcopy(way_points))

# rr_two_link.set_control_mode(ControlTypes.RESOLVED_RATE)
# sequential_trajectories(resolved_rate_control, way_points)
loop.run_until_complete(rr_two_link.shutdown())