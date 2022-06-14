#!/usr/bin/env python
"""
@author Bruce Iverson
"""

from __future__ import annotations
import math

##########################################
####### DYNAMIXEL UNIT CONVERSIONS #######
##########################################
# conversions here may be unique to the XM430-W210 motors. 
# Values taken from here: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/#goal-position
DEGREES_PER_UNIT = 0.08791
RADIANS_PER_UNIT = 0.00153435538
RPM_PER_UNIT = 0.229
RADIANS_PER_SEC_PER_RPM = 2*math.pi/60
MILLI_AMPS_PER_UNIT = 2.69
VOLTS_PER_UNIT = 0.1
PWM_PER_UNIT = 0.113


def convert_raw_position_to_radians(raw:int) -> float:
    """Expects the raw value in dynamixel units"""
    # degrees = raw*DEGREES_PER_UNIT
    # return math.radians(degrees)
    return raw*RADIANS_PER_UNIT


def convert_radians_to_raw_position(radians:float) -> int:
    # return int(math.degrees(radians)/DEGREES_PER_UNIT)
    return int(round(radians/RADIANS_PER_UNIT))


def convert_raw_velocity_to_rpm(raw:int) -> float:
    """Expects the raw value in dynamixel units"""
    return raw*RPM_PER_UNIT
    

def convert_raw_velocity_to_radians_per_second(raw:int) -> float:
    """Expects the raw value in dynamixel units"""
    rpm = convert_raw_velocity_to_rpm(raw)
    return rpm*RADIANS_PER_SEC_PER_RPM


def convert_velocity_radians_to_raw(radians_per_second:float) -> int:
    rpm = radians_per_second/RADIANS_PER_SEC_PER_RPM
    return int(rpm/RPM_PER_UNIT)


def convert_raw_current_to_milliamps(raw:int) -> float:
    """Expects the raw value in dynamixel units"""
    return raw*MILLI_AMPS_PER_UNIT


def convert_current_milliamps_to_raw(ma:float) -> int:
    return int(ma/MILLI_AMPS_PER_UNIT)


def convert_raw_voltage_to_volts(raw:int) -> float:
    """Expects the raw value in dynamixel units"""
    return raw*VOLTS_PER_UNIT


def convert_volts_to_raw(volts:float) -> int:
    return int(volts/VOLTS_PER_UNIT)


def convert_raw_pwm_to_pwm(raw:int) -> float:
    """Expects the raw value in dynamixel units"""
    result = raw*PWM_PER_UNIT    # this should be a percentage
    if result > 100:
        return 100
    elif result < 0:
        return 0
    else: return result


def convert_pwm_to_raw(pwm:float) -> int:
    return int(pwm/PWM_PER_UNIT)
