from __future__ import annotations
from gpiozero import AngularServo			# wrapper for servo motors controlled via raspi GPIO
from time import sleep
from dataclasses import dataclass
import asyncio
import logging
import numpy as np 
import math
import traceback

# maybe useful?
from typing import NoReturn, Coroutine, Optional
from collections import deque

# hardware pwm on GPIO12, 13, 18, 19 for RPi 3 
servo1_pin = 13
servo2_pin = 18

def get_simple_logger(name: str, verbosity=logging.WARNING) -> logging.Logger:
	logger = logging.getLogger(name)
	# Create handlers. These say what to do with items as they get added to the logger
	c_handler = logging.StreamHandler()
	f_handler = logging.FileHandler('file.log')
	c_handler.setLevel(verbosity)
	f_handler.setLevel(verbosity)

	# Create formatters and add it to handlers
	c_format = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
	f_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
	c_handler.setFormatter(c_format)
	f_handler.setFormatter(f_format)

	# Add handlers to the logger
	logger.addHandler(c_handler)
	logger.addHandler(f_handler)

	logger.setLevel(verbosity)
	print(f'Logger level set to: {logger.level}')
	return logger


@dataclass
class RotationalLink():
	"""This class represents one revolute link in a robot arm. Note that nearly all of the math 
	done in this class is work in the actuator space and the joint space"""
	
	# def __init__(self):
	a_iplus1: float					# the length of the link in the x_i direction
	alpha_i_1: float				# the rotation about the x_i-1 axis (degrees)
	d_i: float						# the length of the link in the y_i direction
	# note: theta_i is a property defined in a method below
	actuator: AngularServo				# the motor
	logger: logging.logger
	name: str
	_servo_position_offset = 0		# theta_i = actuator position + _servo_position_offset
	_servo_dir = 1 				# should either be 1 or -1
	# _transformation: np.array = np.array()

	##################################################
	### FOR MAPPING SERVO POSITION INTO THETA_I COORDINATE SYSTEM
	##################################################

	@property
	def _actuator_setpoint(self) -> float:
		return self.actuator.angle
		

	@property
	def theta_i(self) -> float:
		"""This is the actuator set position in the link frame of reference"""
		#  the link rotation about the z_i axis
		return self._actuatorspace_to_jointspace(self.actuator.angle)


	@property
	def min_theta_i(self) -> float:
		return self._actuatorspace_to_jointspace(self.actuator.min_angle)


	@property
	def max_theta_i(self) -> float:
		return self._actuatorspace_to_jointspace(self.actuator.max_angle)


	@property
	def is_at_min(self) -> bool:
		return self._actuator_setpoint < self.actuator.min_angle + 1		# units are degrees


	@property
	def is_at_max(self) -> bool:
		return self._actuator_setpoint > self.actuator.max_angle - 1		# units are degrees


	#########################
	### Private methods
	#########################
	def _actuatorspace_to_jointspace(self, servo_angle: float) -> float:
		"""Returns the equivalent theta_i for an servo angle
		
		
		Angle argument given in degrees"""

		return servo_angle*self._servo_dir + self._servo_position_offset


	def _jointspace_to_actuatorspace(self, theta_i: float) -> float:
		"""Returns the equivalent servo angle for a given theta_i


		Angle argument given in degrees"""

		return theta_i - self._servo_position_offset


	def _get_valid_actuator_value(self, actuator_angle: float) -> float:
		"""Takes the given angle and saturates the value at the min and max servo positions."""

		if actuator_angle > self.actuator.max_angle:
			return self.actuator.max_angle
		elif actuator_angle < self.actuator.min_angle:
			return self.actuator.min_angle
		else: 
			return round(actuator_angle, 1)


	#########################
	### Public methods
	#########################(
	def check_valid_theta(self, proposed_theta:float) -> bool:
		return proposed_theta > self.min_theta_i and proposed_theta < self.max_theta_i
	
	
	def go_to_absolute(self, target_theta: float) -> None:
		"""Sends a pwm command to the servo motor to go to a position
		
		target_theta is the angle given in standard reference frame ("""

		# ensure that we only send values within the normal range
		new_actuator_angle = self._jointspace_to_actuatorspace(target_theta)
		
		new_actuator_angle = self._get_valid_actuator_value(new_actuator_angle)
		
		# print(f'{self.name} set to position: {new_actuator_angle} degrees')
		self.logger.info(f'{self.name} set to position: {new_actuator_angle} deg, given val: {target_theta}')
		self.actuator.angle = new_actuator_angle
	

	def go_to_relative(self, distance: float) -> None:
		"""Distance expected in degrees"""

		new_actuator_angle = self._actuator_setpoint + distance  						# in degrees

		new_actuator_angle = self._get_valid_actuator_value(new_actuator_angle)

		# print(f'{self.name} set to position: {new_actuator_angle} degrees')
		self.logger.info(f'{self.name} set to position: {new_actuator_angle} degrees')
		self.actuator.angle = new_actuator_angle


	async def sweep(self, direction: int=1) -> AutomaticError:
		"""Move the link2 joint from one side of its motion to the other.
		Direction should be either 1 or -1."""
		# main code is here
		try:
			self.logger.info(f'_sweep begin for link {self.name}! Direction: {direction}')
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
			
			self.logger.info(f'_sweep task has concluded {self.name}.')

		except asyncio.CancelledError:
			print(f'Sweep task cancelled!')
		except KeyboardInterrupt:
			self.logger.info(f'Sweep detected keyboard interrupt.')


@dataclass
class CartesianPosition():
	"""Representation of the cartesian position of the end effector, 
	typically in the base reference frame"""
	x: Optional[float] = None
	y: Optional[float] = None

	@property
	def get_as_string(self) -> str:
		return f'x: {self.x} y: {self.y}'
	

class RRTwoLinkArm():
	"""This class represents a simple planar 2 link RR robot arm."""
	
	def __init__(self):

		# general/utility
		self.logger = get_simple_logger("RRTwoLink", verbosity=logging.DEBUG)

		# set up the motors
		s1 = AngularServo(servo1_pin, min_angle=-30, max_angle=240, min_pulse_width=.5/1000, max_pulse_width=2.5/1000)
		s2 = AngularServo(servo2_pin, min_angle=-90, max_angle=180, min_pulse_width=.5/1000, max_pulse_width=2.5/1000)

		# define the linkages in the arm per the textbook standard
		self.link1 = RotationalLink(200, 0, 0, 
									actuator=s1,
									logger=self.logger, 
									name="link 1")

		self.link2 = RotationalLink(200, 0, 0, 
									actuator=s2, 
									logger=self.logger, 
									name="link 2")

		self.ee_position = CartesianPosition()
		self._task: Optional[AsyncioTask] = None


	def go_to_absolute(self, theta_1: float, theta_2: float) -> None:
		"""Move the links to the positions passed (in the joint space)
		
		Values expected in degrees."""

		self.link1.go_to_absolute(theta_1)
		self.link2.go_to_absolute(theta_2)
		self._update_forward_kinematics()


	def go_to_relative(self, dist1: float, dist2: float) -> None:
		"""Move the links by the amout passed (in the joint space)
		
		Values expected in degrees."""

		self.link1.go_to_relative(dist1)
		self.link2.go_to_relative(dist2)
		self._update_forward_kinematics()


	def _check_valid_joint_positions(self, theta1:float, theta2:float) -> bool:
		return self.link1.check_valid_theta(theta1) and self.link2.check_valid_theta(theta2)

	def _check_valid_cartesian_positions(self, x:float, y: float) -> bool:
		if (x**2 + y**2)**0.5 > 400:
			return False
		else: return True


	def go_to_cartesian(self, x: float, y: float) -> None:
		"""This function does the inverse kinematics to move the end effector 
		to a specified location in cartesian space"""

		self._update_forward_kinematics()	# ensure before we move that everything is up to date

		# check that we can actually move to the position
		is_valid = self._check_valid_cartesian_positions(x, y)

		if not is_valid:
			self.logger.error(f'A cartesian command was passed with a point outside of the workspace. X: {x}, Y: {y}')
			return None

		# the solution 0, 0 has infinite solutions, its the only one. 
		if round(x, 1) == 0 and round(y,1) == 0:
			# get the current theta1
			theta1 = self.link1.theta_i
			self.go_to_absolute(theta1, 180)
			return None

		# get parameters for calcs
		L1 = self.link1.a_iplus1
		L2 = self.link2.a_iplus1

		# calculations. Reference: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
		try:
			c2 = (x**2 + y**2 - L1**2 - L2**2)/(2*L1*L2)	# cosine of theta 2
			
			# calculate all possible solutions
			theta2_sol1 = math.acos(c2)					# this is in radians, default for the math package
			theta2_sol2 = -theta2_sol1

			numerator_sol1 = L2*math.sin(theta2_sol1)
			numerator_sol2 = L2*math.sin(theta2_sol2)
			denominator_sol1 = L1 + L2*math.cos(theta2_sol1)
			denominator_sol2 = L1 + L2*math.cos(theta2_sol2)

			theta1_sol1 = math.atan2(y, x) - math.atan2(numerator_sol1, denominator_sol1)
			theta1_sol2 = math.atan2(y, x) - math.atan2(numerator_sol2, denominator_sol2)
		except:
			self.logger.warning(f'Error arose in inverse kinematics calculations.')
			self.loggerk0.debug(f'Traceback: {traceback.print_exc()}')

		# convert to degrees
		theta1_sol1 = round(math.degrees(theta1_sol1), 1)
		theta1_sol2 = round(math.degrees(theta1_sol2), 1)
		theta2_sol1 = round(math.degrees(theta2_sol1), 1)
		theta2_sol2 = round(math.degrees(theta2_sol2), 1)

		self.logger.debug(f'Inverse kinematics solutions: t1 s1: {theta1_sol1}, t2 s1: {theta2_sol1}, t1 s2: {theta1_sol2}, t2 s2: {theta2_sol2}')

		# check that the calculated joint angles are valid/in the joint workspace
		solution_1_is_valid = self._check_valid_joint_positions(theta1_sol1, theta2_sol1)
		solution_2_is_valid = self._check_valid_joint_positions(theta1_sol2, theta2_sol2)
		
		if solution_1_is_valid and solution_2_is_valid:			# find the solution closest to current position
			self.logger.debug(f'Inverse kinematics calculated both solutions valid')

			# check to see which is closer from the current set point and go there
			t1_s1_dist = abs(self.link1.theta_i - theta1_sol1)
			t1_s2_dist = abs(self.link1.theta_i - theta1_sol2)

			t2_s1_dist = abs(self.link2.theta_i - theta2_sol1)
			t2_s2_dist = abs(self.link2.theta_i - theta2_sol2)

			if t1_s1_dist + t2_s1_dist < t1_s2_dist + t2_s2_dist:
				# solution 1 is shorter
				self.logger.debug(f'Solution 1 calculated to be net shorter distance in joint space')
				self.go_to_absolute(theta1_sol1, theta2_sol2)
			else:
				# solution 2 is shorter
				self.logger.debug(f'Solution 2 calculated to be net shorter distance in joint space')
				self.go_to_absolute(theta1_sol2, theta2_sol2)
				
		elif solution_1_is_valid:
			# go to solution 1
			self.logger.debug(f'Solution 1 is only valid solution')
			self.go_to_absolute(theta1_sol1, theta2_sol1)

		elif solution_2_is_valid:
			# go to solution 2
			self.logger.debug(f'Solution 2 is only valid solution')
			self.go_to_absolute(theta1_sol2, theta2_sol2)

		else:
			# neither is valid
			self.logger.error(f'Inverse kinematics has failed to solve for joint space solution to x: {x} and y: {y}.')


	def _update_forward_kinematics(self) -> None:
		"""Updates the estimated end effector position based on the joint positions"""

		# do the thing for link 1
		L1 = self.link1.a_iplus1
		theta1 = math.radians(self.link1.theta_i)		# now we are in radians

		L2 = self.link2.a_iplus1
		theta2 = math.radians(self.link2.theta_i)		# now we are in radians

		x = L1*math.cos(theta1) + L2*math.cos(theta1 + theta2)
		y = L1*math.sin(theta1) + L2*math.sin(theta1 + theta2)
		
		# make these numbers nice
		x = round(x, 1)
		y = round(y, 1)

		# upate the end effector position
		self.ee_position.x = x
		self.ee_position.y = y

		self.logger.debug(f'End effector end position forward kinematics: {self.ee_position.get_as_string}')
		# self.logger.debug(f'theta1: {theta1}, theta2: {theta2}')
		# self.logger.debug(f'x: {x}, y: {y}')


	async def full_workspace_demo(self) -> None:
		try:
			# initialize by going to all minimum positions and set up some vars
			self.link1.go_to_absolute(self.link1.min_theta_i)
			step_res = 15
			sleep_time = .125

			l2_dir = 1
			# sweep forwards
			while self.link1.theta_i < self.link1.max_theta_i - 1:
				await self.link2.sweep(l2_dir)

				self.link1.go_to_relative(step_res)
				sleep(sleep_time)
				l2_dir *= -1

				# if self.link1.is_at_max:
				# 	self.link1.sweep(direction=-1)
				# else:
				# 	self.link1.sweep(direction=1)

			# sweep backwards
			# while self.link2.theta_i > self.link2.min_theta_i + 1:
			# 	self.link2.go_to_relative(-step_res)
			# 	if self.link1.is_at_max:
			# 		self.link1.sweep(direction=-1):
			# 	else:
			# 		self.link1.sweep(direction=1)
				# sleep(sleep_time)

		except KeyboardInterrupt:
			self.logger.info(f'Workspace demo cancelled by keyboard interrupt.')
		except AsyncioCancelledError:
			self.logger.info(f'Workspace demo cancelled.')


if __name__ == "__main__":
	try:
		robot_arm = RRTwoLinkArm()
		direc = 1
		robot_arm.go_to_absolute(-90, -90)	
		sleep(0.5)
        # link1.go_to_absolute(robot_arm.link1.min_theta_i)
		# robot_arm.link1.go_to_absolute(robot_arm.link2.min_theta_i)
	
		sleep_time = 3	
		# https://stackoverflow.com/questions/58774718/asyncio-in-corroutine-runtimeerror-no-running-event-loop
		loop = asyncio.get_event_loop()
		tasks = []
		while 1:
#			loop.run_until_complete(robot_arm.link1.sweep(direc))
			robot_arm.go_to_absolute(0, 0)
			sleep(sleep_time)
			robot_arm.go_to_absolute(180, 180)
			direc *= -1
			sleep(sleep_time)

	except KeyboardInterrupt:

		loop.close()
		print(f'The program has been stopped.')
