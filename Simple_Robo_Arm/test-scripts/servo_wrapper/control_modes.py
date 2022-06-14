from __future__ import annotations
import sys
import logging
import math
import time

sys.path.append(sys.path[0] + '/../..')

import rsl.rr_two_link.robotics.dynamixel_wrapper as dyna_wrapper

if __name__ == "__main__":
    """Test software for basic functionality"""

    dxl_io = dyna_wrapper.wrapper.get_serial_connection()
    motor = dyna_wrapper.wrapper.DynamixelServoWrapper(
        motor_id=1,
        dxl_io=dxl_io,
        min_angle=0,
        max_angle=2*math.pi,
        theta_offset=-math.pi,
        log_level=logging.WARNING
    )
    
    motor.read_simple_state()
