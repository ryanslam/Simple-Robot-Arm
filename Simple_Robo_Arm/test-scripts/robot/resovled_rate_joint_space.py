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

sys.path.append(sys.path[0] + '/../..')
from rsl.rr_two_link.robotics.robot import *


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


def track_trajectory(control_func:function, start:tuple(float), goal:tuple(float)):
	# set up the trajectory
	stop_watch = StopWatch()
	traj = arm._produce_joint_trajectory(start, goal)
	dt = traj.t[1] - traj.t[0]
	for t, q, qd in zip(traj.t, traj.q, traj.qd):
		loop_start = time.monotonic()
		control_func(q, qd)
		
		# time stuff
		if t == 0: stop_watch.start()
		time_elapsed = time.monotonic() - loop_start
		while time_elapsed < dt:
			time.sleep(0.002)
			time_elapsed = time.monotonic() - loop_start

	print(f'Motion took {stop_watch.get_time()} seconds')
	stop_watch.reset()


def sequential_trajectories(control_func:function, way_points:list[tuple[float]]):
	arm.read_motor_states()
	start = arm.Q
	if all([abs(i - j) < 10 for i, j in zip(start, way_points[0])]):		# already near the starting point
		start = way_points.pop(0)

	for goal in way_points:
		track_trajectory(control_func, start, goal)
		start = goal


def resolved_rate_control_law(q, qd):
	"""Takes in the target position and velocity from a trajectory and moves the arm accordingly"""
	global arm
	# q = np.reshape(q, (2,1))
	# qd = np.reshape(qd, (2, 1))

	arm.read_motor_states()

	Kp = 0.4
	position_error = q - arm.Q
	target_speeds = qd + Kp*position_error

	lim = 1											# rad/s
	for joint_vel in qd:
		if joint_vel > lim:
			print(f'Warning! Velocity limit reached.')
			return
	arm.set_joint_velocities(target_speeds)


arm = RRTwoLink(log_level=logging.DEBUG)
arm.torque_enable()

start = (0, 0)
way_points = [start, (pi/2, pi/2), (pi/4, -pi/2)]

arm.set_joint_pose(start)
time.sleep(1)

arm.set_control_mode(ControlTypes.RESOLVED_RATE)
sequential_trajectories(resolved_rate_control_law, way_points)

loop = asyncio.get_event_loop()
loop.run_until_complete(arm.shutdown())