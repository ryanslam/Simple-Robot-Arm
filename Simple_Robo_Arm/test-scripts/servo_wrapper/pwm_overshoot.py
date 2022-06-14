from __future__ import annotations
import sys
import logging
import math
import time


sys.path.append(sys.path[0] + '/../..')

from rsl.rr_two_link.robotics.dynamixel_wrapper.wrapper import DynamixelControlMode
import rsl.rr_two_link.robotics.dynamixel_wrapper as dyna_wrapper

dxl_io = dyna_wrapper.wrapper.get_serial_connection()
motor = dyna_wrapper.wrapper.DynamixelServoWrapper(
    motor_id=1,
    dxl_io=dxl_io,
    min_angle=0,
    max_angle=2*math.pi,
    theta_offset=-math.pi,
    log_level=logging.WARNING
)
motor2 = dyna_wrapper.wrapper.DynamixelServoWrapper(
    motor_id=2,
    dxl_io=dxl_io,
    min_angle=0,
    max_angle=2*math.pi,
    theta_offset=-math.pi,
    log_level=logging.INFO
)

print(f'Normal motion')
motor2.torque_enable()
motor2.set_theta(math.pi*0.95)
motor.torque_enable()
motor.set_theta(math.pi/4)
time.sleep(2)
motor2.torque_disable()

motor2.set_control_mode(DynamixelControlMode.CURRENT)
motor2.torque_enable()
motor2.set_current(1000)

time.sleep(1)

# motor.servo.write_control_table("Position_P_Gain", 300)
# motor.servo.write_control_table("Position_D_Gain", 800)
# motor.servo.write_control_table("Position_I_Gain", 0)
# motor.servo.write_control_table("Feedforward_2nd_Gain", 0)
# motor.servo.write_control_table("Feedforward_1st_Gain", 0)

try:
    while 1:
        time.sleep(0.1)
except KeyboardInterrupt:
    motor.torque_disable()
    motor2.torque_disable()


