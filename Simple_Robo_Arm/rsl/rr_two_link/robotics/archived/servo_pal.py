
# THIS FILE CONTAINS DIFFERENT WRAPPERS FOR DIFFERENT WAYS FOR CONTROLLING SERVO MOTORS, ENABLING SIMPLE SWAPPING AT HIGHER LEVELS
from __future__ import annotations


class SimpleServo():
	"""This class represents one revolute link in a robot arm. Note that nearly all of the math 
	done in this class is work in the actuator space and the joint space"""
	from gpiozero import AngularServo

    def __init__(pin: int, min_angle:float, max_angle:float):
        self.actuator = AngularServo(pin, min_angle=min_angle, max_angle=max_angle)				# the motor

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
	#########################(
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


	def sweep(self, direction: int=1) -> AutomaticError:
		"""Move the link2 joint from one side of its motion to the other.
		Direction should be either 1 or -1."""
		# main code is here
		try:
			self.logger.info(f'_sweep begin! Direction: {direction}')
			# initialize the motor starting angle.
			print(f'Initializing')
			if direction == -1:
				self.go_to_absolute(self.max_theta_i)
			else: 										# direction == 1 is the nominal case and the default here
				self.go_to_absolute(self.min_theta_i)
			sleep(1)
			
			step_res = 5
			sleep_time = 0.125
			# sweep
			def is_done():
				if direction == -1:
					return self.is_at_min 
				else:
					return self.is_at_max

			while not is_done():
				if direction == -1:
					self.go_to_relative(-step_res)
				else:
					self.go_to_relative(step_res)
				sleep(sleep_time)
			
			self.logger.info('_sweep task has concluded.')

		except asyncio.CancelledError:
			print(f'Sweep task cancelled!')
		except KeyboardInterrupt:
			self.logger.info(f'Sweep detected keyboard interrupt.')
