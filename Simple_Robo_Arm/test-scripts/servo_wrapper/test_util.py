from __future__ import annotations
import sys
import asyncio
import time
sys.path.append(sys.path[0]+ '/../..')

import rsl.rr_two_link.robotics.dynamixel_wrapper.wrapper as wrapper

async def single_motor_service(motor:wrapper.DynamixelServoWrapper):
    """Continuously read from the servo motor"""
    servo_rate = 0.25
    while 1:
        # update the motor states. Kinematic model directly interprets from this
        start_time = time.monotonic()
        motor.read_simple_state()
            
        elapsed_time = time.monotonic() - start_time
        sleep_time = servo_rate - elapsed_time
        if sleep_time < 0: 
            print(f'Servo rate is set to faster than the computer can read information from the motors. Elapsed time: {elapsed_time}')
        else:
            await asyncio.sleep(sleep_time)

def start_motor_service(motor):
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        loop = asyncio.get_event_loop()
    return loop.create_task(single_motor_service(motor))

