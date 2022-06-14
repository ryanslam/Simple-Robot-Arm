from __future__ import annotations
import sys
import logging
import math
import time

sys.path.append(sys.path[0] + '/../..')

import rsl.rr_two_link.robotics.dynamixel_wrapper as dyna_wrapper

dxl_io = dyna_wrapper.wrapper.get_serial_connection()
motor = dyna_wrapper.wrapper.DynamixelServoWrapper(
    motor_id=1,
    dxl_io=dxl_io,
    min_angle=0,
    max_angle=2*math.pi,
    theta_offset=-math.pi,
    log_level=logging.DEBUG
)

# motor.torque_disable()
# motor.servo.write_control_table("Homing_Offset", dyna_wrapper.unit_conversions.convert_radians_to_raw_position(0))

# home_offset = motor.servo.read_control_table("Homing_Offset")
# print(f'Homing offset is now: {home_offset}')

# motor.torque_enable()
# print(f'Initializing at theta 0...')
# motor.set_theta(0)
# time.sleep(2)

# print(f'Going to angle 0...')
# motor._set_angle(0)
# time.sleep(2)

# motor.torque_disable()
# print(f'Changing home position to pi/4')
# motor.servo.write_control_table("Homing_Offset", dyna_wrapper.unit_conversions.convert_radians_to_raw_position(math.pi))
# home_offset = motor.servo.read_control_table("Homing_Offset")
# print(f'Homing offset is now: {home_offset}')

# print(f'moving to theta 0')
# time.sleep(0.25)
# motor.torque_enable()
# motor.set_theta(0)
# time.sleep(2)

# # print(f'moving to angle 0')
# # motor.set_angle(0)
# time.sleep(2)
# motor.torque_disable()

motor.torque_disable()
motor.servo.write_control_table("Homing_Offset", dyna_wrapper.unit_conversions.convert_radians_to_raw_position(math.pi))

home_offset = motor.servo.read_control_table("Homing_Offset")
print(f'Homing offset is now: {home_offset}')

motor.torque_enable()
print(f'Going to angle 0...')
motor._set_angle(0)
time.sleep(2)

print(f'Moving to angle math.pi')
motor._set_angle(math.pi)
time.sleep(2)
motor.torque_disable()


motor.servo.write_control_table("Homing_Offset", dyna_wrapper.unit_conversions.convert_radians_to_raw_position())
home_offset = motor.servo.read_control_table("Homing_Offset")
print(f'Homing offset is now: {home_offset}')
