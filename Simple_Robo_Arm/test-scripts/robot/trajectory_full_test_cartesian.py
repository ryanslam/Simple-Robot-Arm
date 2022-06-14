#!/usr/bin/env python
"""
@author Bruce Iverson
"""

from __future__ import annotations
import sys
import logging
import time
import sys
import copy

sys.path.append(sys.path[0] + '/../..')
from rsl.rr_two_link.robotics.robot import *


def run(local_way_points:list[tuple[float,float]]):
    global rr_two_link
    rr_two_link.read_motor_states()
    start = rr_two_link.X
    if all([abs(i - j) < 10 for i, j in zip(start, way_points[0])]):		# already near the starting point
        start = way_points.pop(0)

    async def main_loop():
        for point in way_points:
            rr_two_link.set_cartesian_goal_position(point, True)
            while rr_two_link.has_trajectory:
                time.sleep(0.25)
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main_loop())

rr_two_link = RRTwoLink(log_level=logging.INFO)
rr_two_link.torque_enable()
start = (400, 0)
way_points = [start, (200, 0)]

# way_points = [start, (200, 200), (200, 0), start]
# way_points = [start, (200, 200), start]

rr_two_link.set_cartesian_goal_position(start)
time.sleep(1)
# run(copy.deepcopy(way_points))

# rr_two_link.set_cartesian_goal_position(way_points[1], True)

for t, q in zip(rr_two_link._traj.t, rr_two_link._traj.q):
    loop_start = time.monotonic()
    rr_two_link.set_joint_pose(q)    
    time_elapsed = time.monotonic() - loop_start
    if time_elapsed > rr_two_link._servo_rate:
        print(f'warning! loop not executing fast enough')
    while time_elapsed < rr_two_link._servo_rate:
        time.sleep(0.002)
        time_elapsed = time.monotonic() - loop_start


# rr_two_link.set_cartesian_goal_position(start)
# time.sleep(1)
# rr_two_link.set_control_mode(ControlTypes.JOINT_PID)
# run(copy.deepcopy(way_points))

loop = asyncio.get_event_loop()
loop.run_until_complete(rr_two_link.shutdown())