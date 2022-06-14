from __future__ import annotations
import sys
import logging
import math
import time
import statistics

sys.path.append(sys.path[0] + '/../..')

from rsl.rr_two_link.robotics.dynamixel_wrapper.wrapper import DynamixelControlMode
import rsl.rr_two_link.robotics.dynamixel_wrapper as dyna_wrapper



dxl_io = dyna_wrapper.wrapper.get_serial_connection()

# dxl_io.port_handler.ser.set_low_latency_mode(True)

motor = dyna_wrapper.wrapper.DynamixelServoWrapper(
    motor_id=1,
    dxl_io=dxl_io,
    min_angle=0,
    max_angle=2*math.pi,
    theta_offset=-math.pi,
    log_level=logging.WARNING
)


def timing_test(func, args=[]):
    times = []
    for _ in range(10):
        start = time.monotonic()
        func(*args)
        end = time.monotonic()
        t = end - start
        times.append(t)
    mean = round(statistics.mean(times), 4)
    print(f'Mean time for execute function {func.__name__}: {mean}, variance: {statistics.variance(times)}')
    return mean


timing_test(motor.torque_enable)
t1 = timing_test(motor.set_theta, [0])       # result: about 0.0158
t2 = timing_test(motor.read_simple_state)    # result: about 0.0319
min_period = t1 + t2
max_freq = 1/min_period

print(f'Min period: {min_period}, max frequency: {max_freq}')

