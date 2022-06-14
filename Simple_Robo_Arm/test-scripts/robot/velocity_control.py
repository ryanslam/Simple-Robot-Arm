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
motor.set_control_mode(dyna_wrapper.wrapper.DynamixelControlMode.VELOCITY)
motor.torque_enable()
speeds = (1, math.pi, 7)
# note: confirmed that math.pi speed is 0.5 rev/s
for _ in range(2):
    for speed in speeds:
        print(f'Moving at speed {speed}')
        motor.set_velocity(speed)
        time.sleep(2)
    speeds = [-s for s in speeds]
motor.torque_disable()

