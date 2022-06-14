#!/usr/bin/env python
"""
@author Bruce Iverson
"""

from __future__ import annotations
import sys
import logging
import time
import numpy as np
import sys

sys.path.append(sys.path[0] + '/../..')
from rsl.rr_two_link.robotics.robot import *
from rsl.rr_two_link.robotics.controllers import JointPositionPID


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


goal = (0, 0) # theta, radians

rr_two_link = RRTwoLink(log_level=logging.INFO)
rr_two_link.torque_enable()

# rr_two_link.start_service(new_servo_rate=0.1)
rr_two_link.set_control_mode(ControlTypes.ACTUATOR_POSITION_CONTROL)
rr_two_link.set_joint_pose((np.pi/2, np.pi/2))
time.sleep(1)

rr_two_link.read_motor_states()
rr_two_link.set_control_mode(ControlTypes.JOINT_PID)

time.sleep(1)
rr_two_link.control_manager.active_controller.set_gains(0, 60, 0, 30)
rr_two_link.control_manager.active_controller.set_gains(1, 35, 0, 0)
rr_two_link.control_manager.active_controller.set_target(goal, None)
stop_watch = StopWatch()
stop_watch.start()

servo_rate = 0.16
while stop_watch.get_time() < 6:
	loop_start = time.monotonic()
	rr_two_link.read_motor_states()
	milli_amps = rr_two_link.control_manager.active_controller.control_law(rr_two_link.Q)
	rr_two_link.set_currents(milli_amps)
	elapsed_time = time.monotonic() - loop_start
	sleep_time = servo_rate - elapsed_time
	if sleep_time < 0:
		print(f'Servo rate is set to faster than the computer can read information from the motors. Elapsed time: {elapsed_time}')
	else:
		time.sleep(sleep_time)

    # while time.monotonic() - loop_start < servo_rate:
    #     time.sleep(0.001)

rr_two_link.torque_disable()

# time.sleep(0.5)
# loop = asyncio.get_event_loop()
# loop.run_until_complete(rr_two_link.shutdown())