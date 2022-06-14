#!/usr/bin/env python
"""
@author Bruce Iverson
"""


from __future__ import annotations
import sys
import logging
import math
import time

sys.path.append(sys.path[0] + '/../..')
import rsl.rr_two_link.robotics.dynamixel_wrapper as dyna_wrapper


dxl_io = dyna_wrapper.wrapper.get_serial_connection()
motor = dyna_wrapper.wrapper.DynamixelServoWrapper(
    motor_id=2,
    dxl_io=dxl_io,
    min_angle=0,
    max_angle=2*math.pi,
    theta_offset=-math.pi,
    log_level=logging.DEBUG
)

motor.torque_enable()
print(f'Initializing...')
motor.set_theta(0)
time.sleep(2)


motor.torque_disable()
motor.set_control_mode(dyna_wrapper.wrapper.DynamixelControlMode.CURRENT)
motor.torque_enable()
time.sleep(0.1)
for _ in range(2):
    for mA in (10, 100, 1000, 0):
        motor.set_current(mA)
        time.sleep(3)


    