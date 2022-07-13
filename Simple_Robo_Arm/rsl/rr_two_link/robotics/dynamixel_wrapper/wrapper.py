#!/usr/bin/env python
"""
@author Bruce Iverson
"""

from __future__ import annotations
from dataclasses import dataclass, field
import os
from typing import Any, Optional
import dynio as dynamixel
from enum import IntEnum, Enum
import time
import math
import logging
import asyncio
import sys

from .unit_conversions import *
import rsl.rr_two_link.util as util


class PowerState(Enum):
    ON = "TORQUE_ENABLED"
    OFF = "TORQUE_DISABLED"


class CommunicationState(IntEnum):
    GOOD = 1
    ERROR = 255


class DynamixelConnectionError(Exception):
    pass


@dataclass
class MotorLimits:
    min_angle:float
    max_angle:float
    max_velocity:float
    max_current:float


DEFAULT_MOTOR_LIMITS = MotorLimits(
    0,
    2*math.pi,
    7.91,                       # 7.91366521 rad/sec 
    1193*MILLI_AMPS_PER_UNIT   # mA
)


@dataclass
class DynamixelState:
    """These objects are constructed with the values read out of the dynamixel motor to be the main parameters, and property methods defined for unit conversions"""
    state:CommunicationState
    position_raw:float                                                 # raw, .008 deg/unit
    velocity_raw:Optional[float] = field(default=None)          # raw, .229 rpm
    current_raw:Optional[float] = field(default=None)           # 2.69 mA, ranging from 0 to 1,193 
    temperature:Optional[float] = field(default=None)
    timestamp:float = field(default_factory=time.monotonic)     # want to use a monotonic clock here so age never times out in unexpected ways

    @property 
    def age(self) -> float:
        """The age of the status in seconds"""
        return time.monotonic() - self.timestamp

    @property
    def expired(self) -> bool:
        return self.age > .1

    @property
    def position(self) -> float:
        return convert_raw_position_to_radians(self.position_raw)

    @property 
    def velocity_rpm(self) -> float:
        """Returns the velocity in RPM"""
        if self.velocity_raw is None: return None
        return convert_raw_velocity_to_rpm(self.velocity_raw)

    @property 
    def velocity(self) -> float:
        """Returns the velocity in radians/sec"""
        if self.velocity_raw is None: return None
        return convert_raw_velocity_to_radians_per_second(self.velocity_raw)

    @property
    def current(self) -> float:
        """Returns the current in milliamps"""
        if self.current_raw is None: return None
        return convert_raw_current_to_milliamps(self.current_raw)

    @property
    def simple_repr(self) -> str:
        s = f"position: {util.prettify_radians(self.position)};"
        if self.velocity is not None:
            s += f" velocity {util.prettify_radians(self.velocity)}/sec;"
        if self.temperature is not None:
            s += f' temp {self.temperature} C;'
        if self.current is not None:
            s += f" current_raw {round(self.current,2)} mA"
        return s

    @property
    def simple_repr_raw_values(self) -> str:
        position = f"position: {util.prettify_radians(self.position)};"
        return f"{position} velocity_raw {self.velocity_raw}; temp_raw {self.temperature} C; current_raw {self.current_raw}"


class DynamixelControlMode(IntEnum):
    CURRENT = 0
    VELOCITY = 1
    POSITION = 3
    EXTENDED_POSITION = 4
    CURRENT_BASED_POSITION = 5
    PWM = 6                         # THIS IS VOLTAGE


def get_serial_connection(baud_rate=57600) -> dynamixel.dxl.DynamixelIO:
    # setup
    if os.name == "nt": port = "COM4"
    else: port = '/dev/ttyUSB0'

    try:
        dxl_io = dynamixel.dxl.DynamixelIO(
            device_name=port,
            baud_rate=57600
            ) # your port for U2D2 or other serial device
        return dxl_io
    except:
        print(f'FAILED T0 ACQUIRE SERIAL PORT!!!')
        print(f'port: {port}')
        raise DynamixelConnectionError


def new_xm430_W210(dxl_id:int, communication_obj:dynamixel.dxl.DynamixelIO):
    """Returns a new DynamixelMotor object for an XM430-W210. Note that the JSON file
    does not store actual values, but rather is a mapping of strings to the address and number of bytes
    in a place in the control table."""

    # obj = pkg_resources.resource_filename(__name__, "../json/XM430-W210.json"),   # 

    json_path = os.path.abspath('.') + '/robotics/dynamixel_wrapper/dynamixel_json/XM430-W210.json'
    motor = dynamixel.dxl.DynamixelMotor(
        dxl_id,                                                                 # integer
        communication_obj,                                                      # DynamixelIO object
        json_path,
        protocol=2,                                                             # unsure if protocol 1 is supported for XM series, always using 2.
        # control_table_protocol=None,                                          # this gets set inside the class later by the protocol argument
        )

    motor.torque_disable()
    motor.set_position_mode()
    kp = 100 #rad/s
    motor.write_control_table("Position_P_Gain", kp)
    return motor, kp


def eeprom_write_protected(func):
    """Ensure that the servo has torque disabled before writing values"""
    async def wrapper(*args):
        #ARGS[0] IS USUALLY SELF


        append = '.'
        if len(args[1:]) > 0:
            append = f'; args: {args[1:]}'
        
        obj = args[0]
        try:
            logger.debug(f'Attempting command: {obj.name.name} {func.__name__.upper()}' + append)
        except AttributeError:
            logger.debug(f'Attempting command: {obj.name} {func.__name__.upper()}' + append)

        result = await func(*args)
        await asyncio.sleep(0.02)
        return result
    return wrapper


class DynamixelServoWrapper:
    """A wrppaer for a dynamixel motor object that provides some additional layers of functionality,
    including: reading and writing to the dynamixel control table with functions not set by the dynamixel-controller library,
    giving a consolidated way of reading multiple values at once, timestamps on read values, and logging/debugging tools.
    
    Useful links: 
        Simple example of usage: https://pypi.org/project/dynamixel-controller/
        Slightly more detailed example: https://github.com/UGA-BSAIL/dynamixel-controller/blob/moreJSON/docs.md
        The dynamixel motor code: https://github.com/UGA-BSAIL/dynamixel-controller/blob/moreJSON/dynio/dynamixel_controller.py
        
    Theta and its offset: while the dynamixel library has a homing offset, that offset only affects the read position, not a written position. 
    Therefore instead, the Homing_Offset is ignored and unused."""
    
    def __init__(
        self, 
        motor_id:int, 
        dxl_io:dynamixel.dxl.DynamixelIO,
        min_angle:float=0, 
        max_angle:float=2*math.pi, 
        theta_offset:float=-math.pi, 
        server_log_handler=None,
        log_level:int=logging.WARNING):

        """All angles expected in radians."""
        # general/utility   
        self.motor_id = motor_id
        self.logger = util.get_simple_logger(f'servo_{motor_id}', log_level)
        if server_log_handler is not None:
            self.logger.addHandler(server_log_handler)
        # motor stuff
        self._theta_offset = theta_offset
        self.power_status = PowerState.OFF
        self.control_mode = DynamixelControlMode.POSITION
        self.servo, kp = new_xm430_W210(motor_id, dxl_io)       # defaults to position control, torque disabled
        self.gains:dict[str, float] = {'kp': kp, 'ki': 0, 'kd': 0}
        self._motor_limits = DEFAULT_MOTOR_LIMITS

        model = self.servo.read_control_table("Model_Number")
        fw_version = self.servo.read_control_table("Firmware_Version")
        self.logger.info(f'New dynamixel servo created! model: {model}, firmware version: {fw_version}')
        self.logger.debug(f'Default motor limits: {DEFAULT_MOTOR_LIMITS}')
        self.logger.debug(f'Default motor speed units: {convert_velocity_radians_to_raw(DEFAULT_MOTOR_LIMITS.max_velocity)}')
        self.set_position_limit(min_angle, max_angle)
        self.set_velocity_limit()
        self.print_motor_limits()
        self._position_limited_by_hardware = False

        self.latest_state:DynamixelState = self.read_full_state()
        self.set_theta(0)
        # self.torque_enable()

        """In order to calculate the angle theta of the servo for use with link dh parameter kinematic model of serial chain manipulator,
        the dynamixel motor angle given by the dynio library in degrees is mapped to radians, and the below number is added to that to give theta."""

    #############################################
    ################## UTILITY ##################
    #############################################
    @property
    def near_max_position(self) -> bool:
        """Within 1 degree"""
        return self.latest_state.position > self._motor_limits.max_angle - math.radians(1)

    @property
    def near_min_position(self) -> bool:
        """Within 1 degree"""
        return self.latest_state.position < self._motor_limits.min_angle + math.radians(1)
    
    @property
    def theta(self) -> float:
        """In radians"""
        if self.control_mode in (DynamixelControlMode.CURRENT, DynamixelControlMode.VELOCITY) and self.motor_id == 1:
            return self.latest_state.position + self._theta_offset - math.pi
        else:
            return self.latest_state.position + self._theta_offset

    @property
    def max_theta(self) -> float:
        return self._motor_limits.max_angle + self._theta_offset

    @property
    def min_theta(self) -> float:
        return self._motor_limits.min_angle + self._theta_offset

    @property
    def torque_is_enabled(self) -> bool:
        return self.power_status == PowerState.ON
    
    def _convert_theta_to_angle(self, theta:float) -> float:
        angle = theta - self._theta_offset
        self.logger.debug(f'Converted theta {theta} to angle {angle} rad')
        return angle

    def check_if_theta_is_valid(self, theta:float) -> bool:
        less_than_max = theta <= self.max_theta
        return theta >= self.min_theta and less_than_max

    def check_if_velocity_is_valid(self, v:float) -> bool:
        """Expecting v in radians/sec"""
        too_high = v > self._max_limits.velocity
        too_low = v < self._min_limits.velocity
        if not too_high and not too_low: return True
        else:
            self.logger.warning(f'Velocity check failed. Velocity lims: \
                {util.prettify_radians(self._max_limits.velocity)}/sec \
                {util.prettify_radians(self._max_limits.velocity)}/sec')

    def print_motor_limits(self):
        # read out the limits stored on the motor (helpful for debugging)
        if self.logger.getEffectiveLevel() > 10:
            return 
        self.logger.debug(f'MOTOR LIMITS')
        my_dict:dict = {
            'Homing_Offset': {'func':convert_raw_position_to_radians, 'units': 'rad'}, 
            'Max_Voltage_Limit': {'func':convert_raw_voltage_to_volts, 'units': 'volts'},
            'Min_Voltage_Limit': {'func':convert_raw_voltage_to_volts, 'units': 'volts'},
            'PWM_Limit': {'func':convert_raw_pwm_to_pwm, 'units': '%'},
            'Current_Limit': {'func':convert_raw_current_to_milliamps, 'units': 'mA'},
            'Velocity_Limit': {'func':convert_raw_velocity_to_radians_per_second, 'units': 'radians/sec'}, 
            'Max_Position_Limit': {'func':convert_raw_position_to_radians, 'units': 'radians'}, 
            'Min_Position_Limit': {'func':convert_raw_position_to_radians, 'units': 'radians'}
            }

        for item_name, v in my_dict.items():
            raw = self.servo.read_control_table(item_name)
            func = v['func']
            unit:str = v['units']
            self.logger.debug(f'{item_name} raw: {raw}; converted: {round(func(raw), 3)} {unit}.')

    #############################################
    ################## READING ##################
    #############################################
    def _helper_demangle_data(self, x:int, data_size:int=32) -> int:
        """The dynamixel library messes up negative values due to improper bit math (Two's Complement)
        https://github.com/UGA-BSAIL/dynamixel-controller/blob/58a956c16c612ea3e33df955397c4857d1043c07/dynamixel_sdk/robotis_def.py
        In the MAKE_WORD function. This file is in a wrapper package for dynamixel_sdk, and that function matches the 
        most up to date Robotis distribution of the SDK. Function is deprecated."""

        if x > 2 << (data_size - 2):   # overflowed in the negative dir
            x -= 2 << (data_size - 1)
        return x

    def _get_position(self) -> int:
        raw = self.servo.get_position()
        return self._helper_demangle_data(raw)

    def _get_velocity(self) -> int:
        """This functionality was not provided by the dynio library. Note that the units here are weird, 0.229 RPM"""
        v = self.servo.read_control_table("Present_Velocity")
        return self._helper_demangle_data(v, 32)

    def _get_temperature(self) -> int:
        """This functionality was not provided by the dynio library. deg celcius"""
        return self.servo.read_control_table("Present_Temperature")

    def _get_current(self) -> int:
        """This helper demangles the read current"""
        return self._helper_demangle_data(self.servo.get_current(), 16)

    def _get_control_mode(self) -> DynamixelControlMode:
        raw = self.servo.read_control_table("Operating_Mode")
        mode = DynamixelControlMode(raw)
        self.logger.debug(f'Fetched motor control mode: {mode.name}')
        self.control_mode = mode
        return mode

    def read_simple_state(self):
        position_raw = self._get_position()
        velocity_raw = self._get_velocity()
        new_state = DynamixelState(
            CommunicationState.GOOD,
            position_raw,
            velocity_raw,
        )
        self.latest_state = new_state
        self.logger.debug(f'New state --> {self.latest_state.simple_repr}')

        if self._position_limited_by_hardware:
            self._position_protection()
        return self.latest_state

    def read_full_state(self):
        """Reads in the current_raw state"""
        position_raw = self.servo.get_position()
        current_raw = self._get_current()
        velocity_raw = self._get_velocity()
        temperature = self._get_temperature()
        self.latest_state = DynamixelState(
            CommunicationState.GOOD,
            position_raw,
            velocity_raw,
            current_raw,
            temperature
        )

        self.logger.debug(f'New state --> {self.latest_state.simple_repr}')
        
        if self._position_limited_by_hardware:
            self._position_protection()
        return self.latest_state

    def _position_protection(self):
        """This exists to keep the first motor from going too far when in non position control modes (ie resolved rate)"""
        # check the operating mode
        if not self.control_mode in (DynamixelControlMode.POSITION, DynamixelControlMode.EXTENDED_POSITION) and self.torque_is_enabled:
            position_deadband = math.pi/6
            
            # we're doing all operations here in radians in the actuator space (NOT theta, NOT the weird dynamixel units)
            too_far_forwards = self.latest_state.position > self._motor_limits.max_angle - position_deadband
            too_far_backwards = self.latest_state.position < self._motor_limits.min_angle + position_deadband
            
            # check the speed? for now, no, instead we just keep a high dead_band
            if too_far_forwards or too_far_backwards:
                self.logger.error(f'Detected that the motor has entered an area where it is in danger of damaging iteself or the robot. Disabling motor. Current position: {util.prettify_radians(self.latest_state.position)}')
                self.torque_disable()
        
    #############################################
    ################## WRITING ##################
    #############################################
    def set_position_limit(self, min_angle:float, max_angle:float):
        """Velocity limit expected in radians/sec. Default is 330 raw. 
        Note that it is important for mathematical reasons to preserve the limits in the original units given, 
        not raw to prevent issues converting back and forth."""
        min_raw = convert_radians_to_raw_position(min_angle)
        max_raw = convert_radians_to_raw_position(max_angle)
        self._motor_limits.min_angle = min_angle
        self._motor_limits.max_angle = max_angle

        self.logger.debug(f'Setting minimum position to angle {util.prettify_radians(self._motor_limits.min_angle)} radians, {min_raw} units')
        self.servo.write_control_table('Min_Position_Limit', min_raw)
        self.logger.debug(f'Setting maximum position to {util.prettify_radians(self._motor_limits.max_angle)} radians, {max_raw} units')
        self.servo.write_control_table('Max_Position_Limit', max_raw)

    def set_velocity_limit(self, vel_limit:float=7.914):
        """Velocity limit expected in radians/sec. Default is 330 raw. 
        Note that it is important for mathematical reasons to preserve the limits in the original units given, 
        not raw to prevent issues converting back and forth."""

        turned_off_torque:bool = False
        if self.torque_is_enabled:
            self.logger.warning(f'[SET_VELOCITY_LIMIT] Temporarily disabling torque in order to write to EEPROM are of servo memory.')
            self.torque_disable()
            turned_off_torque = True

        vel_limit_raw = convert_velocity_radians_to_raw(vel_limit)
        self._motor_limits.max_velocity = vel_limit
        self.logger.info(f'Setting velocity limit to {util.prettify_radians(self._motor_limits.max_velocity)}/s, {vel_limit_raw} units')
        self.servo.write_control_table('Velocity_Limit', vel_limit_raw)

        # validate via reflectance that this was successful
        real_limit_raw = self.servo.read_control_table('Velocity_Limit')
        real_limit = convert_raw_velocity_to_radians_per_second(real_limit_raw)
        self.logger.info(f'The velocity limit is now: {util.prettify_radians(real_limit)}/s, raw: {real_limit_raw}')
        self._motor_limits.max_velocity = vel_limit
        
        # turn the torque back on if we turned it off
        if turned_off_torque:
            self.torque_enable()
            self.logger.info(f'Torque re enabled.')
    
    def set_current_limit(self, max_current:float):
        """Limit expected in mA. Default is 1193 raw. 
        Note that it is important for mathematical reasons to preserve the limits in the original units given, 
        not raw to prevent issues converting back and forth."""
        self._motor_limits.max_current = max_current
        raw_value = convert_current_milliamps_to_raw(max_current)

        self.logger.debug(f'Setting minimum position to angle {max_current} mA, {raw_value} units')
        self.servo.write_control_table('Current_Limit', raw_value)

    def set_motor_gains(self, kp=800, kd=0):
        """Set the motor gains."""

        kp_raw, kd_raw = [int(item) for item in (kp, kd)]
        self.logger.info(f'Setting motor internal controller P gain to {kp_raw} (default is 800), D to {kd_raw}')
        self.servo.write_control_table('Position_P_Gain', kp_raw)
        self.servo.write_control_table('Position_D_Gain', kd_raw)

        # validate via reflectance that this was successful
        real_kp = self.servo.read_control_table('Position_P_Gain')
        if real_kp == kp_raw:
            self.logger.info(f'Kp successfully changed! is now: {kp_raw}')
            self.gains['kp'] = kp_raw
            self.gains['kd'] = kd_raw
        else:
            self.logger.warning(f'Kp change failed! Kp is still: {real_kp}')

    def _set_position(self, new_position:int):
        self.logger.debug(f'Position set to {new_position}')
        # enforce limits
        min_raw = convert_radians_to_raw_position(self._motor_limits.min_angle)
        max_raw = convert_radians_to_raw_position(self._motor_limits.max_angle)
        if new_position > max_raw: 
            self.logger.warning(f'Given position is larger than the position limit {max_raw}. Setting positiong to the limit instead.')
            new_position = max_raw
        if new_position < min_raw:
            self.logger.warning(f'Given position is smaller than the position limit {min_raw}. Setting positiong to the limit instead.')
            new_position = min_raw
        self.logger.debug(f'Setting position: {new_position} units')
        self.servo.set_position(int(new_position))
    
    def _set_angle(self, new_angle:float):
        """Angle expected in radians"""
        self.logger.debug(f'Setting position using conversion from radians, value: {util.prettify_radians(new_angle)}')
        self._set_position(convert_radians_to_raw_position(new_angle))

    def set_theta(self, theta:float):
        """Sets the motor position using a calculation to match theta"""
        if not self.check_if_theta_is_valid(theta):
            self.logger.warning(f'Function SET_THETA received an invalid value! {theta} rad. Limits: {self.min_theta} {self.max_theta}')
            self.logger.debug(f'Function SET_THETA received an invalid value! {theta} rad. Limits: {util.prettify_radians(self.min_theta)} {util.prettify_radians(self.max_theta)}')
        else:
            self.logger.debug(f'Setting angle from theta {util.prettify_radians(theta)}')
            angle = self._convert_theta_to_angle(theta)
            self._set_angle(angle)
    
    def set_velocity(self, new_vel:float):
        """Velocity expected in radians/second"""
        write_value= convert_velocity_radians_to_raw(new_vel)
        self.servo.set_velocity(write_value)
        self.logger.debug(f'Velocity set to {new_vel} rad/s, write: {write_value}')
    
    def set_current(self, goal_current:float):
        """Input expexted in mA. This will be limited by the Current_Limit setting. Maximum Current_Limit setting is about 3.209 A
        
        This operation is particularly prone to failure, so success is gauged via reflectance."""
        
        self.logger.debug(f'Setting goal current to {round(goal_current, 2)} mA')
        write_value = convert_current_milliamps_to_raw(goal_current)
        if abs(goal_current) > self._motor_limits.max_current:
            self.logger.error(f'Invalid value {goal_current} mA passed to set_current function of dynamixel motor. Write val: {write_value} outside of bounds')
        else:
            self.servo.write_control_table("Goal_Current", write_value)

    def set_control_mode(self, incoming_mode:DynamixelControlMode) -> bool:
        """Set the control mode on the servo motor, validate the change via reflectance to ensure that 
        software knowledge of the control mode is always accurate.
        
        Note that torque must be disabled in order to change the motor control mode as it is in a protected section of the control table.."""

        # Read more here: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/#operating-mode 
        if type(incoming_mode) != DynamixelControlMode:
            raise ValueError(f"Invalid parameter passed to SET_CONTROL_MODE: {incoming_mode.name}")
        elif incoming_mode == self._get_control_mode():
            self.logger.debug(f'Set control mode exiting as motor is already in mode {incoming_mode.name}')
            return
        self.logger.info(f'Attempting to set control mode to {incoming_mode.name}, value: {incoming_mode.value}')
        if incoming_mode == DynamixelControlMode.CURRENT:
            """DYNAMIXEL only controls current_raw(torque) regardless of speed and position. 
            This mode is ideal for a gripper or a system that only uses current_raw(torque) control or a system that 
            has additional velocity/position controllers."""
            self.servo.write_control_table("Operating_Mode", incoming_mode.value)
            # if goal_current is not None:
            #     self.servo.write_control_table("Goal_Current", goal_current)
        elif incoming_mode == DynamixelControlMode.VELOCITY:
            """This mode controls velocity. This mode is identical to the Wheel Mode(endless) from existing DYNAMIXEL. 
            This mode is ideal for wheel-type robots."""
            self.set_velocity(0)
            # self.servo.set_velocity_mode()
            self.servo.write_control_table("Operating_Mode", incoming_mode.value)
        elif incoming_mode == DynamixelControlMode.POSITION:
            """This mode controls position. This mode is identical to the Joint Mode from existing DYNAMIXEL. 
            Operating position range is limited by the Max Position Limit(48) and the Min Position Limit(52). 
            This mode is ideal for articulated robots that each joint rotates less than 360 degrees."""
            self.servo.set_position_mode()
        elif incoming_mode == DynamixelControlMode.EXTENDED_POSITION:
            """This mode controls position. This mode is identical to the Multi-turn Position Control from 
            existing DYNAMIXEL. 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for multi-turn 
            wrists or conveyer systems or  a system that requires an additional reduction gear. 
            Note that Max Position Limit(48), Min Position Limit(52) are not used on Extended Position Control Mode."""
            self.servo.write_control_table("Operating_Mode", incoming_mode.value)
        elif incoming_mode == DynamixelControlMode.CURRENT_BASED_POSITION:
            """This mode controls both position and current_raw(torque). Up to 512 turns are 
            supported(-256[rev] ~ 256[rev]). This mode is ideal for a system that requires both 
            position and current_raw control such as articulated robots or grippers."""
            self.servo.write_control_table("Operating_Mode", incoming_mode.value)
        elif incoming_mode == DynamixelControlMode.PWM:
            """This mode directly controls PWM output. (Voltage Control Mode)"""
            self.servo.write_control_table("Operating_Mode", incoming_mode.value)

        success = incoming_mode == self._get_control_mode()
        if not success:
            self.logger.warning(f'SET_CONTROL_MODE failed! Incoming: {incoming_mode.name}, current mode: {self.control_mode.name}')
        else:
            self.logger.info(f'The control mode has been successfully changed to {incoming_mode.name}.')
            self.control_mode = incoming_mode
            self.read_simple_state()
        return success

    def torque_enable(self):
        self.servo.torque_enable()
        # check if we succeeded
        if self.servo.read_control_table("Torque_Enable") == 1:
            self.logger.info(f'Motor torque has been enabled.')
            self.power_status = PowerState.ON
        else:
            self.logger.error(f'ENABLE method was called but motor torque is still disabled.')
            self.power_status = PowerState.OFF

    def torque_disable(self):
        self.servo.torque_disable()
        # check if we succeeded
        if self.servo.read_control_table("Torque_Enable") == 0:
            self.logger.info(f'Motor torque has been disabled.')
            self.power_status = PowerState.OFF
        else:
            self.logger.error(f'DISABLE method was called but motor torque is still enabled.')
            self.power_status = PowerState.ON

    #############################################
    ################ COROUTINES #################
    #############################################
    async def discrete_sweep(self, direction:int, step_resolution=2*math.pi/20, sleep_time=0.15):
        """Move the joint from one side of its motion to the other. Direction should be either 1 or -1.
        Note that this task assumes that a separate task is running to read the state."""
        try:
            self.logger.debug(f'SWEEP begin! Direction: {direction}')
			# initialize the motor starting angle.
            if direction == -1:
                target_theta = self.max_theta
            elif direction == 1: 										# direction == 1 is the nominal case and the default here
                target_theta = self.min_theta
            else:
                self.logger.error(f'Sweep failed, direction {direction} must be either 1 or -1')

            # initialize by moving to the start position
            start_time = time.monotonic()
            self.set_theta(target_theta)
            while 1:
                self.logger.error(f'In loop')
                error = abs(target_theta - self.theta)
                if error < math.radians(5): 
                    self.logger.debug(f'break')
                    break
                if time.monotonic() - start_time > 5: 
                    self.logger.error(f'Sweep initial movement has timed out. \
                        Latest angle: {util.prettify_radians(self.latest_state.position)}, age {self.latest_state.age} sec, \
                        theta: {util.prettify_radians(self.theta)}, min: {util.prettify_radians(self.min_theta)}, max: {util.prettify_radians(self.max_theta)}')

                    raise asyncio.CancelledError
                    
                await asyncio.sleep(0.5)

            self.logger.debug(f'Sweep has arrived near to the initial position')

			# sweep
            def is_done():
                if direction == -1:
                    return self.near_min_position 
                else:
                    return self.near_max_position

            while not is_done():
                if direction == -1:
                    target_theta -= step_resolution
                    if target_theta < self.min_theta: target_theta = self.min_theta
                    self.set_theta(target_theta)
                else:
                    target_theta += step_resolution
                    if target_theta > self.max_theta: target_theta = self.max_theta
                    self.set_theta(target_theta)
                await asyncio.sleep(sleep_time)

            self.logger.debug(f'discrete sweep task has concluded.')

        except asyncio.CancelledError:
            self.logger.info(f'Sweep task cancelled!')
        except KeyboardInterrupt:
            self.logger.info(f'Sweep detected keyboard interrupt.')

    async def smooth_sweep(self, direction:int, velocity:float):
        """Move the joint from one side of its motion to the other. Direction should be either 1 or -1.
        
        """

        try:
            if not self.check_if_velocity_is_valid(velocity):
                self.logger.error(f'Function SLOW_SWEEP received a velocity that is outside of the current bounds.')
                raise asyncio.CancelledError

            self.logger.info(f'SWEEP begin! Direction: {direction}')
			# initialize the motor starting angle.
            if direction == -1:
                target_theta = self.max_theta
            elif direction == 1: 										# direction == 1 is the nominal case and the default here
                target_theta = self.min_theta
            else:
                self.logger.error(f'Sweep failed, direction {direction} must be either 1 or -1')

            # initialize by moving to the start position
            start_time = time.monotonic()
            self.set_theta(target_theta)
            while 1:
                error = abs(target_theta - self.theta)
                if error < math.radians(3): break
                if time.monotonic() - start_time > 5: 
                    self.logger.error(f'Sweep initial movement has timed out. \
                        Latest angle: {util.prettify_radians(self.latest_state.position)}, age {self.latest_state.age} sec, \
                        theta: {util.prettify_radians(self.theta)}, min: {util.prettify_radians(self.min_theta)}, max: {util.prettify_radians(self.max_theta)}')
                    raise asyncio.CancelledError
                await asyncio.sleep(0.1)

            self.logger.debug(f'Sweep has arrived near to the initial position')
            self.set_control_mode(DynamixelControlMode.VELOCITY)            
            
			# sweep
            def is_done():
                if direction == -1:
                    return self.near_min_position 
                else:
                    return self.near_max_position

            self.set_velocity(velocity*direction)

            while not is_done():
                await asyncio.sleep(0.1)

            self.set_control_mode(DynamixelControlMode.POSITION)
            self.logger.info(f'_sweep task has concluded.')

        except asyncio.CancelledError:
            self.logger.info(f'Sweep task cancelled!')
        except KeyboardInterrupt:
            self.logger.info(f'Sweep detected keyboard interrupt.')
