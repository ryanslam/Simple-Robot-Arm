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
from numpy.linalg import LinAlgError
import roboticstoolbox as rtb
import spatialmath
import sys
import copy

sys.path.append(sys.path[0] + '/../..')
from rsl.rr_two_link.robotics.robot import *


arm = RRTwoLink(log_level=logging.INFO)
arm.torque_enable()


class StopWatch:
	def __init__(self):
		self._start_time:Optional[float] = None

	@property
	def is_running(self) -> bool:
		return self._start_time
	
	def get_time(self) -> float:
		return time.monotonic() - self._start_time

	def start(self):
		self._start_time = time.monotonic()
	
	def reset(self):
		self._start_time = None


def track_trajectory(start:tuple(float), goal:tuple(float)):
	
	traj = arm._produce_cartesian_trajectory(start, goal)._traj
	dt = traj.t[1] - traj.t[0]; print(f'dt is: {dt}')
	
	stop_watch = StopWatch()
	stop_watch.start()

	for t, q in zip(traj.t, traj.q):
		loop_start = time.monotonic()
		# q, success, reason = arm._ikine_analytic(x)
		arm.set_joint_pose(q)
		# time stuff
		time_elapsed = time.monotonic() - loop_start
		if time_elapsed > dt:
			print(f'warning! loop not executing fast enough')
		# print(f'Time: {stop_watch.get_time()}, t: {t}. Difference {round(stop_watch.get_time() - t)}')
		while time_elapsed < dt:
			time.sleep(0.002)
			time_elapsed = time.monotonic() - loop_start

	print(f'Motion took {stop_watch.get_time()} seconds')
	stop_watch.reset()


def sequential_trajectories(way_points:list[tuple[float]]):
	global arm
	arm.read_motor_states()
	start = arm.X
	wp = copy.deepcopy(way_points)
	if all([abs(i - j) < 10 for i, j in zip(start, wp[0])]):		# already near the starting point
		start = wp.pop(0)
	for goal in wp:
		track_trajectory(start, goal)
		start = goal

start = (400, 0)
way_points = [start, (200, 0)]
# way_points = [start, (200, 200), (200, 0), start]
# way_points = [start, (200, 200), start]

arm.set_cartesian_goal_position(start)
time.sleep(1)

# print(F'FIRST MOTION')
# sequential_trajectories(way_points)
# time.sleep(1)

print(F'SECOND MOTION WITH JOINT PID')
arm.set_control_mode(ControlTypes.JOINT_PID)
time.sleep(0.25)
arm.set_joint_pose((0, 0))
time.sleep(0.25)
arm.set_joint_pose((90, 90))
time.sleep(0.25)
arm.set_joint_pose((0, 0))

sequential_trajectories(list(reversed(way_points)))
time.sleep(1)

print(F'THIRD MOTION')
arm.set_control_mode(ControlTypes.ACTUATOR_POSITION_CONTROL)
time.sleep(0.25)
sequential_trajectories(way_points)

loop = asyncio.get_event_loop()
loop.run_until_complete(arm.shutdown())