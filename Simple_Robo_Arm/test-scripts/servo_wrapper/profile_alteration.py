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
motor2 = dyna_wrapper.wrapper.DynamixelServoWrapper(
    motor_id=2,
    dxl_io=dxl_io,
    min_angle=0,
    max_angle=2*math.pi,
    theta_offset=-math.pi,
    log_level=logging.WARNING
)

print(f'Normal motion')
motor2.set_theta(math.pi*0.95)
motor2.torque_enable()
motor.torque_enable()
motor.set_theta(0)

# motion_type = motor.servo.read_control_table("Moving_Status")
# print(f'Motion type: {motion_type}')

time.sleep(1)
motor.set_theta(math.pi/2)
time.sleep(1)
motor.torque_disable()


# profile_vel = motor.servo.read_control_table("Profile_Velocity")
# print(f'Profile_Velocity initialized at: {profile_vel}')
# profile_accel = motor.servo.read_control_table("Profile_Acceleration")
# print(f'Profile_Acceleration initialized at: {profile_accel}')


# motor.servo.write_control_table("Profile_Velocity", dyna_wrapper.unit_conversions.convert_radians_to_raw_position(0))
# motor.servo.write_control_table("Profile_Acceleration", dyna_wrapper.unit_conversions.convert_radians_to_raw_position(0))


# profile_vel = motor.servo.read_control_table("Profile_Velocity")
# print(f'Profile_Velocity is now: {profile_vel}')
# profile_accel = motor.servo.read_control_table("Profile_Acceleration")
# print(f'Profile_Acceleration is now: {profile_accel}')


motor.servo.write_control_table("Position_P_Gain", 300)
motor.servo.write_control_table("Position_D_Gain", 800)
motor.servo.write_control_table("Position_I_Gain", 0)
motor.servo.write_control_table("Feedforward_2nd_Gain", 0)
motor.servo.write_control_table("Feedforward_1st_Gain", 0)
print(f'new motion')
motor.torque_enable()
motor.set_theta(0)
# motion_type = motor.servo.read_control_table("Moving_Status")
# print(f'Motion type: {motion_type}')
time.sleep(1)
motor.set_theta(math.pi/2)
time.sleep(1)

try:
    while 1:
        time.sleep(0.1)
except KeyboardInterrupt:
    motor.torque_disable()
    motor2.torque_disable()


