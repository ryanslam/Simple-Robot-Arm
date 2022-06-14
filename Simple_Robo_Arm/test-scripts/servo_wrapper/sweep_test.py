from __future__ import annotations
import sys
import logging
import math
import time
import asyncio

sys.path.append(sys.path[0] + '/../..')

import rsl.rr_two_link.robotics.dynamixel_wrapper as dyna_wrapper
from test_util import *


async def discrete_sweep_n_times(servo:dyna_wrapper.wrapper.DynamixelServoWrapper, n=3):
    try:
        for _ in range(n):
            await servo.discrete_sweep(-1)
            await servo.discrete_sweep(1)
    except asyncio.CancelledError:
        print(f'Sweep forever cancelled!')
    except KeyboardInterrupt:
        print(f'Sweep forever detected keyboard interrupt.')


async def smooth_sweep_n_times(servo:dyna_wrapper.wrapper.DynamixelServoWrapper, n=3):
    try:
        v = math.pi/2 #rad/s
        for _ in range(n):
            await servo.smooth_sweep(-1, v)
            await servo.smooth_sweep(1, v)
    except asyncio.CancelledError:
        print(f'Sweep forever cancelled!')
    except KeyboardInterrupt:
        print(f'Sweep forever detected keyboard interrupt.')


if __name__ == "__main__":
    """Test software for basic functionality"""
    import os
    import argparse
    parser = argparse.ArgumentParser()
    # Add an argument
    parser.add_argument('--demo', type=str, required=False)
    # Parse the argument
    args = parser.parse_args()
    mode = args.demo

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
    try:
        if mode is None: mode = 'smooth-sweep'
        print(f'Starting routine {mode}!')
        start_motor_service(motor)
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            loop = asyncio.get_event_loop()
        if mode == 'sweep':
            task = loop.run_until_complete(discrete_sweep_n_times(motor, 3))
        elif mode == "smooth-sweep":
            task = loop.run_until_complete(smooth_sweep_n_times(motor, 3))            
        elif mode == 'workspace':
            pass
        elif mode == 'current_raw':
            # motor.set_control_mode(dyna_wrapper.wrapper.DynamixelControlModes.CURRENT)
            motor.set_current(500)
    finally:
        time.sleep(0.2)
        motor.torque_disable()