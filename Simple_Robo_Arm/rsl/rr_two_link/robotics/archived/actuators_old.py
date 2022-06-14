# THIS FILE CONTAINS DIFFERENT WRAPPERS FOR DIFFERENT WAYS FOR CONTROLLING SERVO MOTORS, ENABLING SIMPLE SWAPPING AT HIGHER LEVELS
from __future__ import annotations
from gpiozero import gpiozero.AngularServo
import asyncio

class Actuator:
	def __init__(self):
		# all units in degrees
		# These assume non-continuous rotation servos
		self.max_angle: float = 360
		self.min_angle: float = 0
		self.position: float = 0
		


class SimpleServo():
	"""This class represents one revolute link in a robot arm. Note that nearly all of the math 
	done in this class is work in the actuator space and the joint space"""

    def __init__(pin: int, min_angle:float, max_angle:float):
        self.actuator: AngularServo = gpiozero.AngularServo(pin, min_angle=min_angle, max_angle=max_angle)				# the motor

	##################################################
	### FOR MAPPING SERVO POSITION INTO THETA_I COORDINATE SYSTEM
	##################################################

	@property
	def _actuator_setpoint(self) -> float:
		return self.actuator.angle
		

	@property
	def is_at_min(self) -> bool:
		return self._actuator_setpoint < self.actuator.min_angle + 0.5		# units are degrees


	@property
	def is_at_max(self) -> bool:
		return self._actuator_setpoint > self.actuator.max_angle - 0.5		# units are degrees


	#########################
	### Private methods
	#########################
	def _get_valid_actuator_angle(self, angle_position: float) -> float:
		"""Takes the given angle and saturates the value at the min and max servo positions."""

		if angle_position > self.actuator.max_angle:
			return self.actuator.max_angle
		elif angle_position < self.actuator.min_angle:
			return self.actuator.min_angle
		else: 
			return angle_position


	#########################
	### Public methods
	#########################
	def go_to_absolute(self, target_angle: float) -> None:
		"""Sends a pwm command to the servo motor to go to a position
		
		target_theta is the angle given in standard reference frame"""

		# ensure that we only send values within the normal range
		new_actuator_angle = self._get_valid_actuator_value(target_angle)
		self.actuator.angle = new_actuator_angle
		# self.logger.debug(f'{self.name} set to position: {new_actuator_angle} degrees')
	

	def go_to_relative(self, distance: float) -> None:
		"""Distance expected in degrees"""

		new_actuator_angle = self._actuator_setpoint + distance  						# in degrees
		new_actuator_angle = self._get_valid_actuator_value(new_actuator_angle)

		# print(f'{self.name} set to position: {new_actuator_angle} degrees')
		self.actuator.angle = new_actuator_angle
		# self.logger.debug(f'{self.name} set to position: {new_actuator_angle} degrees')



# set_velocity_mode
# DynamixelMotor.set_velocity_mode(goal_current=None)
# Sets the motor to run in velocity (wheel) mode and sets the goal current if provided

# set_position_mode
# DynamixelMotor.set_position_mode(min_limit=None,
#                                  max_limit=None,
#                                  goal_current=None)
# Sets the motor to run in position (joint) mode and sets the goal current if provided. If position limits are not specified, the full range of motion is used instead

# set_extended_position_mode
# DynamixelMotor.set_extended_position_mode(goal_current=None)
# Sets the motor to run in extended position (multi-turn) mode

# set_velocity
# DynamixelMotor.set_velocity(velocity)
# Sets the goal velocity of the motor

# set_acceleration
# DynamixelMotor.set_acceleration(acceleration)
# Sets the goal acceleration of the motor

# set_position
# DynamixelMotor.set_position(position)
# Sets the goal position of the motor

# set_angle
# DynamixelMotor.set_angle(angle)
# Sets the goal position of the motor with a given angle in degrees

# get_position
# DynamixelMotor.get_position()
# Returns the motor position

# get_angle
# DynamixelMotor.get_angle()
# Returns the motor position as an angle in degrees

# get_current
# DynamixelMotor.get_current()
# Returns the current motor load

# torque_enable
# DynamixelMotor.torque_enable()
# Enables motor torque

# torque_disable
# DynamixelMotor.torque_disable()
# Disables motor torque