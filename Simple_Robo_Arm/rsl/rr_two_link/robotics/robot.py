#!/usr/bin/env python
"""
@author Bruce Iverson
"""

# general utilities
from __future__ import annotations
from dataclasses import dataclass
from time import sleep
import asyncio
import logging
import traceback				# useful for debugging
from enum import Enum

# maybe useful?
from typing import NoReturn, Optional, Collection, Tuple, List, Type

from dynio.dynamixel_controller import DynamixelIO
import matplotlib.pyplot as plt
# from matplotlib.backends.backend_agg import FigureCanvasAgg
# from matplotlib.figure import Figure

# mathematics
import numpy as np
import math
import time 
import roboticstoolbox as rtb
import spatialmath

# modules/libraries from this project
from .controllers import Cartesian2DPosition, ControllerManager, ControlTypes, JointPosition, MathematicalSpaces, ResolvedRate
from .dynamixel_wrapper.wrapper import get_serial_connection, DynamixelServoWrapper, DynamixelControlMode, PowerState
from rsl.rr_two_link.util import get_simple_logger, prettify_radians


class Trajectory:
	"""A class that contains all the data assiciated with a trajectory, a few flags giving meta information, and providing a 
	simple interface for iterating over trajectory.
	
	Note that s and sd are used throughout this class to represent position and velocity respectively. This is done as these both could be 
	either joint position/velocities or cartesian position/velocities."""

	def __init__(self, traj:rtb.tools.trajectory.Trajectory, control_space:MathematicalSpaces, trajectory_space:MathematicalSpaces):
		self._traj:rtb.tools.trajectory.Trajectory = traj
		self._cur_index:int = 0
		self._start_time:float = None
		self.dt = self._traj.t[1] - self._traj.t
		self.space:MathematicalSpaces = trajectory_space		# the space in which the trajectory is generated
		self.control_space:MathematicalSpaces = control_space	# the space into which the trajectory has been mapped, if any

	@property
	def t(self) -> np.ndarray:
		"""The theoretical time in the trajectory that the current position and velocity is given for. In seconds"""
		return self._traj.t

	@property
	def s(self) -> np.ndarray:
		"""The position (either joint or cartesian)"""
		return self._traj.q
	
	@property
	def sd(self) -> np.ndarray:
		"""The velocity (either joint or cartesian)"""
		return self._traj.qd
	
	@property
	def clock_time(self) -> float:
		if self._start_time is None: return None
		return time.monotonic() - self._start_time
		
	@property
	def done(self) -> bool:
		if self._cur_index >= len(self.t):
			return True
		else: return False

	def get_current_setpoints(self) -> tuple[float, float, float]:
		"""Returns the time, position, and velocity of the trajectory."""
		t = self.t[self._cur_index]
		s = self.s[self._cur_index]
		sd = self.sd[self._cur_index]
		return t, s, sd

	def step(self) -> np.ndarray:
		"""Advances to the next step in the trajectory, and returns the position"""
		if self._start_time is None: self._start_time = time.monotonic()
		t, s, sd = self.get_current_setpoints()
		self._cur_index += 1
		return t, s, sd


class JointType(Enum):
	REVOLUTE = "REVOLUTE"
	PRISMATIC = "PRISMATIC"


@dataclass
class LinkDHParameters:
	"""DH parameters and other information for a joint/link. Leave the DH parameter that will be actuated as "None".
	
	Most of these parameters are used for robot kinematics
	In theory it may make sense to use the symbolic library for the actuated DH parameter but there is no use case.
	d --> link offset
	alpha --> link twist
	a --> link length
	offset --> joint variable offset"""

	joint_type:JointType
	d:float
	alpha:float
	a:float
	q:float
	joint_limits:np.ndarray[float]
	position_limited_by_hardware:bool = False	# a flag passed to the DynamixelServoWrapper object that triggers software limits on position


def produce_rtb_DHRobot_model(name:str, link_dh_info:Collection[LinkDHParameters]) -> rtb.DHRobot:
	"""A means conforming to the module standards of contructing instances of Denavit-Hatenberg defined robot class using Peter Corkes robotics toolbox.
	
	The naming convention for each link is link_x where x is an 0 indexed integer representing link number starting from the base frame.
	Note that this class could be expanded to include mass, friction, and other dynamic properties

	Information from the robotics toolbox about the expected inputs for RevoluteDH class
	:param d: kinematic - link offset
	:type d: float
	:param alpha: kinematic - link twist
	:type alpha: float
	:param a: kinematic - link length
	:type a: float
	:param offset: kinematic - joint variable offset
	:type offset: float

	:param qlim: joint variable limits [min, max]
	:type qlim: float ndarray(1,2)
	:param flip: joint moves in opposite direction
	:type flip: bool
	"""

	# Loop over the link info, switch/case the JointType, and append rtb.RevoluteDH type links to the list
	all_links = []
	for i, dh_params in enumerate(link_dh_info):
		if dh_params.joint_type == JointType.REVOLUTE:
			link = rtb.RevoluteDH(
				d=dh_params.d, alpha=dh_params.alpha, a=dh_params.a, 
				qlim=dh_params.joint_limits, name='link_' + str(i))

		elif dh_params.joint_type == JointType.PRISMATIC:
			raise ValueError(f'Pristmatic joints are not yet supported!')
		else:
			raise ValueError(f'Something is terribly wrong with the joint type of some link!')
		all_links.append(link)

	model = rtb.DHRobot(all_links, name=name)
	model.addconfiguration('q0', [0,0], unit='rad')		# more poses can be added easily in this fashion
	return model


class DynamixelSerialChainManipulator:
	"""The idea behind this class is that a multitude of different robots beyond RR_TWO_LINK 
	joint_types should be a Collection of strings either R or P"""
	def __init__(self, name, link_dh_info:tuple[LinkDHParameters], log_level, servo_log_level=logging.WARNING):
		self.name = name
		self.logger = get_simple_logger(self.name, verbosity=log_level)
		
		self._link_dh_info = link_dh_info
		self.kinematic_model = produce_rtb_DHRobot_model(name, link_dh_info)
		self.n_links:int = len(link_dh_info)

		self._task: Optional[asyncio.Task] = None
		self._service_task: Optional[asyncio.Task] = None
		self.control_manager:ControllerManager = ControllerManager()	# initializes to the actuator position control
		self._servo_rate = 0.16											# the rate at which the motor state gets read and controller writes to the motor. 
		# Typical read time is about 0.0605 sec and write is 0.0286. If servo rate is faster than the sum of these, problems are likely to arise. 
		self._traj:Optional[Trajectory] = None
		
		self._motor_io:DynamixelIO = get_serial_connection()
		self._actuators:list[DynamixelServoWrapper] = []				# these get initialized in the async task "connect_to_motors"
		self._initialize_all_motors(log_level=logging.WARNING)
		self._task: Optional[asyncio.Task] = None

	@property
	def has_task(self) -> bool:
		return self._task is not None and not self._task.done()

	@property
	def has_trajectory(self) -> bool:
		return self._traj is not None

	@property
	def power_status(self) -> PowerState:
		statuses = [a.power_status for a in self._actuators]
		on = all([status == PowerState.ON for status in statuses])
		off = all([status == PowerState.OFF for status in statuses])
		if on: return PowerState.ON
		elif off: return PowerState.OFF
		else:
			# Mixed! this can occur but is an edge case. 
			# Typically occurs when software limits kick in for one of the actuators
			self.logger.warning(f'Robot detected that one of the motors is OFF, and others are ON. Disabling torques.')
			self.torque_disable()
			self.logger.debug(f'Mixed power statuses')
			return PowerState.OFF

	@property
	def on(self) -> bool:
		return self.power_status == PowerState.ON

	def reconnect(self):
		"""Connect to the dynamixel U2D2 and the dynamixel motors"""
		self.logger.info(f'Reconnecting to all dynamixel devices...')
		self._motor_io = get_serial_connection()
		self._initialize_all_motors()

	def _initialize_all_motors(self, log_level=logging.WARNING) -> DynamixelServoWrapper:
		"""Note that this has not been tested with prismatic joints"""
		for i, dh_params in enumerate(self._link_dh_info):
			motor_id = i + 1	# shouldn't this 0 index??? it works like this but maybe a bug?
			if dh_params.joint_type == JointType.REVOLUTE:
				new_motor = self._produce_rotational_motor(motor_id, log_level)
			elif dh_params.joint_type == JointType.PRISMATIC:
				new_motor = self._produce_linear_motor(motor_id)
			else:
				raise ValueError('Must pass valid joint type')
			self._actuators.append(new_motor)

	def _produce_rotational_motor(self, motor_id, log_level) -> DynamixelServoWrapper:
		actuator = DynamixelServoWrapper(
			motor_id=motor_id,
			dxl_io=self._motor_io,
			min_angle=0,
			max_angle=2*math.pi,
			theta_offset=-math.pi,
			# server_log_handler = server_log_handler,	# this will direct the logger to emit the logs to the client
			log_level=log_level
		)
		return actuator

	def _produce_linear_motor(self, motor_id):
		return None

	def set_motor_gains(self, motor_id:int, kp:float, kd:float):
		if motor_id not in (0, 1):
			self.logger.error(f'Received unexpected motor id')
			return
		self._actuators[motor_id].set_motor_gains(kp, kd)
		
	######################################################
	########### Controllers and motion ###########
	######################################################
	def set_control_mode(self, mode:ControlTypes) -> bool:
		# set the mode for the actuators
		actuator_mode = ControlTypes.get_actuator_mode(mode)
		self.logger.info(f'Setting actuator mode to {actuator_mode.name} for incoming control mode {mode.name}')
		orig_power_state = self.power_status
		if orig_power_state == PowerState.ON:
			self.torque_disable()
		results = [a.set_control_mode(actuator_mode) for a in self._actuators]
		
		# kp = 100 #rad/s
		# [a.servo.write_control_table("Position_P_Gain", kp) for a in self._actuators]

		if orig_power_state == PowerState.ON:
			self.torque_enable()

		self.control_manager.set_active_controller(mode)
		if self.control_manager.active_controller.control_space == MathematicalSpaces.JOINT:
			self.control_manager.active_controller.set_target(self.Q, np.array([0, 0]))	# leads to more natural transistions
		else:
			self.control_manager.active_controller.set_target(self.X, np.array([0, 0]))	# leads to more natural transistions
		# self.logger.debug(f'Controller {self.control_manager.active_controller_name.name} has new setpoint: {[prettify_radians(i) for i in self.control_manager.active_controller.get_setpoints()]}')

		if not all(results):
			self.logger.info(f'Control mode change to {mode.name} failed')
			return False
		self.logger.info(f'Control mode successfully changed to {mode.name}')
		return True

	def _produce_general_trajectory(self, 
		start:tuple[float], 
		goal:tuple[float], 
		total_time:float, 
		trajectory_space:MathematicalSpaces, 
		control_space:MathematicalSpaces
		) -> Trajectory:

		"""This is generalized to work with either cartesian positions or with joint positions 
		for the start and the goal, so long as they are 1 dimensional vectors."""
		# set up the trajectory using the robotics toolbox
		dt = self._servo_rate
		n_steps = int(total_time/dt) + 1
		time_vector = np.linspace(0, total_time, num=n_steps)
		self.logger.debug(f'Building a trajectory from {start} to {goal} over {total_time} seconds, with {n_steps} steps.')
		traj_array = rtb.tools.trajectory.jtraj(start, goal, time_vector)
		trajectory = Trajectory(traj_array, control_space, trajectory_space)
		return trajectory

	def produce_joint_trajectory(self, q0:tuple[float], q1:tuple[float], traj_time=None) -> Trajectory:
		"""Make a time based trajectory in the joint space from one pose to another."""
		distances = [abs(i-j) for i, j in zip(q0, q1)]
		max_dist = max(distances)
		if traj_time is None or traj_time < 0.9:
			print(traj_time)
			total_time = round(10*max_dist*6/np.pi + 2)/10
		else: 
			total_time = traj_time
		cntrl_spce = self.control_manager.active_controller.control_space
		if cntrl_spce == MathematicalSpaces.CARTESIAN:
			self.logger.error(f'The active controller for the robot is a CARTESIAN controller. Joint space trajectories with cartesian controllers are not currently supported as there is little reason.')
			return None
		traj:Trajectory = self._produce_general_trajectory(self.Q, q1, total_time, trajectory_space=MathematicalSpaces.JOINT, control_space=cntrl_spce)
		return traj

	######################################################
	########### Methods for handling actuators ###########
	######################################################
	@property
	def Q(self) -> JointPosition:
		"""The current joint angles in radians as a (n, 1) vector"""
		return np.array([motor.theta for motor in self._actuators])

		# @property
	
	@property
	def X(self) -> Cartesian2DPosition:
		"""The current cartesian position of the end effector"""
		# make these numbers nice ish for printing/UI display purposes, they should be in mm
		x, y = self._rtb_fkine(self.Q)
		x = round(x, 3)
		y = round(y, 3)
		# upate the end effector position
		return np.array([x, y])

	def _rtb_fkine(self, q:JointPosition) -> Cartesian2DPosition:
		se3 = self.kinematic_model.fkine(q)
		X = np.array([se3.t[0], se3.t[1]])
		return X

	def _joint_pose_is_valid(self, q:JointPosition) -> bool:
		"""Ensure that setpoints are in the workspace, below motor limits."""
		if len(q) != self.n_links:
			raise ValueError('The number of angles input must match the number of links')
		results = []
		for angle, actuator in zip(q, self._actuators):
			results.append(actuator.check_if_theta_is_valid(angle))
		if all(results):
			return True
		else:
		    self.logger.debug(f'Validating robot pose failed. Angles: {[prettify_radians(t) for t in q]}. Min limits: {[prettify_radians(a.min_theta) for a in self._actuators]}, Max limits: {[prettify_radians(a.max_theta) for a in self._actuators]}')
	
	def torque_enable(self): 
		self.logger.info(f'Motor torques enabling...')
		[a.torque_enable() for a in self._actuators]
	
	def torque_disable(self):
		self.logger.info(f'Motor torques disabling...')
		[a.torque_disable() for a in self._actuators]

	def set_joint_pose(self, q:JointPosition, use_traj=False, traj_time=None) -> bool:
		"""Move the links to the positions passed (in the joint space)

		Values expected in radians."""
        # validate the input
		if not len(q) == self.n_links:
			self.logger.error(f'[SET_JOINT_POSITION] recieved a vector of incorrect length: {q}. n_links: {self.n_links}')
			return
		if self.power_status == PowerState.OFF:
			self.logger.error(f'[SET_JOINT_POSE] Cannot set position while servos are disabled.')
			return
		if not self._joint_pose_is_valid(q):
			self.logger.warning(f'[SET_JOINT_POS] Joint positions {[prettify_radians(t) for t in q]} were invalid.')
			return

		if use_traj:
			self.logger.info(f'[SET_JOINT_POS] Making joint trajectory for target({[prettify_radians(t) for t in q]}); ({[round(math.degrees(t)) for t in q]}) deg Time: {traj_time}.')
			self._traj = self.produce_joint_trajectory(self.Q, q, traj_time)
			return

		self.logger.debug(f'[SET_JOINT_POSE] Moving joint angles to({[prettify_radians(t) for t in q]}); ({[round(math.degrees(t)) for t in q]}) deg.')
		if self.control_manager.active_controller_name == ControlTypes.ACTUATOR_POSITION_CONTROL:
			[a.set_theta(theta) for a, theta in zip(self._actuators, q)]
		# elif self.control_manager.active_controller_name in (ControlTypes.JOINT_PID_TORQUE, ControlTypes.FEED_FORWARDS_VELOCITY_CONTROL):
		elif self.control_manager.active_controller.control_space == MathematicalSpaces.JOINT:
			self.control_manager.active_controller.set_target(q)
			self.logger.debug(f'Controller {self.control_manager.active_controller_name.name} has new setpoint: {[prettify_radians(i) for i in self.control_manager.active_controller.get_setpoints()]}')
		elif self.control_manager.active_controller.control_space == MathematicalSpaces.CARTESIAN:
			# calculate the equivalent cartesian position for the given set of joint angles
			X = self._rtb_fkine(q)
			self.control_manager.active_controller.set_target(X)
		else: raise(TypeError)

	def set_joint_velocities(self, qd:JointPosition) -> bool:
		"""Updates the simulated model and writes the joint speeds to the actuators.
		
		Values expected in radians/s"""
		# update the robotics toolbox model and perform input validations
		if not len(qd) == self.n_links:
			self.logger.error(f'[SET_JOINT_SPEEDS] recieved a vector of incorrect length: {qd}. n_links: {self.n_links}')
			return
		if not self.power_status:
			self.logger.error(f'[SET_JOINT_SPEEDS] Cannot write speeds while servos are disabled.')
			return
		if 1: # self._joint_pose_is_valid(qd):
			# if not all([abs(val) < 0.05 for val in qd]): 
			self.logger.debug(f'[SET_JOINT_SPEEDS] attempting write speeds ({[prettify_radians(t) for t in qd]}/sec); ({[round(math.degrees(t)) for t in qd]}) deg/sec.')
			[a.set_velocity(theta_d) for a, theta_d in zip(self._actuators, qd)]
		else:
			self.logger.warning(f'[SET_JOINT_SPEEDS] Joint positions {[prettify_radians(t) for t in qd]} were invalid.')

	def set_currents(self, mA:Collection[float]) -> bool:
		"""Updates the setpoint for the actuator voltage, writes to actuators"""

		# if not all([abs(ma) < 3.5 for ma in mA]):
		self.logger.debug(f'[SET_CURRENTS] Setting ({[val for val in mA]}) milli amps.')
		[a.set_current(mA[i]) for i, a in enumerate(self._actuators)]

	def read_motor_states(self):
		"""Reads the position and velocity of the motor"""
		[a.read_simple_state() for a in self._actuators]

	async def shutdown(self):
		self.set_joint_pose([0 for _ in range(self.n_links)])
		await asyncio.sleep(0.5)
		self.torque_disable()
		if self._task:
			self._task.cancel()
		if self._service_task:
			self._service_task.cancel()


class RRTwoLink(DynamixelSerialChainManipulator):
	"""This class represents a simple planar 2 link RR robot arm.
	
	Class holds a kinematic model and can read/write to the actuators."""
	def __init__(self, server_log_handler=None, log_level=logging.INFO):

		link_dh_info = (
			LinkDHParameters(
				JointType.REVOLUTE,
				d=0,
				alpha=0,
				a=200,
				q=None,
				joint_limits=np.array([-math.pi, math.pi]),
				position_limited_by_hardware=True
			),
			LinkDHParameters(
				JointType.REVOLUTE,
				d=0,
				alpha=0,
				a=200,
				q=None,
				joint_limits=np.array([-math.pi, math.pi])
			)
		)

		super().__init__(
			'rr_two_link',
			link_dh_info,
			log_level=log_level
			)
		
		if server_log_handler is not None:
			self.logger.addHandler(server_log_handler)
		self.doing_demo = False

	######################################################
	########### Kinematic and position helpers ###########
	######################################################
	# @property
	# def X(self) -> np.ndarray:
		"""HEY I MOVED THIS UP TO THE DYNAMIXEL ROBOT CLASS. LEAVING IT HERE FOR A SEC BUT CAN PROBABLY TAKE IT OUT"""
	# 	"""Forward kinematics estimated end effector position based on the joint positions in cartesian space (millimeters) as a (2, 1) vector"""
	# 	# do the thing for link 1
	# 	x, y = self._forward_kinematics(self.Q)
	# 	# make these numbers nice ish for printing/UI display purposes, they should be in mm
	# 	x = round(x, 3)
	# 	y = round(y, 3)
	# 	# upate the end effector position
	# 	return np.array([x, y])

	def _get_jacob0(self, Q:np.ndarray) -> np.ndarray:
		"""Return the jacobian matrix in the base frame for a given joint pose (in radians), mm"""

		theta1, theta2 = Q
		L2 = self._link_dh_info[1].a
		x, y = self._forward_kinematics(Q)
		return np.array([
			[-y, -L2*np.sin(theta1 + theta2)],
			[x, L2*np.cos(theta1 + theta2)]])

	def _get_jacob0_inv(self, Q:np.ndarray) -> tuple[np.ndarray, bool]:
		"""Returns the inverted jacobian in the base frame for the current joint pose, and a flag indicating whether calculating the matrix was successful.
		
		This method uses the (Moore-Penrose) pseudo-inverse of a matrix."""
		
		try:
			J_inv = np.linalg.pinv(self._get_jacob0(Q))
			return J_inv, True
		except np.linalg.LinAlgError:
			self.logger.error(f'Singularity detected while taking jacobian inverse, matrix is not invertible. Pose: {prettify_radians(Q)}')
			return None, False

	def _helper_link1_forward_kinematics(self, q:np.ndarray) -> np.ndarray:
		x = self._link_dh_info[0].a*math.cos(q[0])
		y = self._link_dh_info[0].a*math.sin(q[0])
		return np.array([x, y])
	
	def _forward_kinematics(self, q:np.ndarray) -> np.ndarray:
		theta1, theta2 = q
		link_1_pos = self._helper_link1_forward_kinematics(q)
		x = link_1_pos[0] + self._link_dh_info[1].a*math.cos(theta1 + theta2)
		y = link_1_pos[1] + self._link_dh_info[1].a*math.sin(theta1 + theta2)
		
		# make these numbers nice ish for printing/UI display purposes, they should be in mm
		x = round(x, 1)
		y = round(y, 1)
		# self.logger.debug(f'End effector position from forward kinematics: {x}, {y}')
		# upate the end effector position
		return np.array([x, y])

	def _helper_ikine_analytic_get_possible_sols(self, point:np.ndarray) -> tuple[tuple[np.ndarray, np.ndarray], bool, str]:
		try:
			x, y = point
			c2 = (x**2 + y**2 - self._link_dh_info[0].a**2 - self._link_dh_info[1].a**2)/(2*self._link_dh_info[0].a*self._link_dh_info[1].a)	# cosine of theta 2
			
			# calculate all possible solutions
			theta2_sol1 = math.acos(c2)						# this is in radians, default for the math package
			theta2_sol2 = -theta2_sol1

			numerator_sol1 = self._link_dh_info[1].a*math.sin(theta2_sol1)
			numerator_sol2 = self._link_dh_info[1].a*math.sin(theta2_sol2)
			denominator_sol1 = self._link_dh_info[0].a + self._link_dh_info[1].a*math.cos(theta2_sol1)
			denominator_sol2 = self._link_dh_info[0].a + self._link_dh_info[1].a*math.cos(theta2_sol2)

			theta1_sol1 = math.atan2(y, x) - math.atan2(numerator_sol1, denominator_sol1)
			theta1_sol2 = math.atan2(y, x) - math.atan2(numerator_sol2, denominator_sol2)
			self.logger.debug(f'[INKINE] Inverse kinematics solutions: t1 s1: {prettify_radians(theta1_sol1)}, t2 s1: {prettify_radians(theta2_sol1)}, t1 s2: {prettify_radians(theta1_sol2)}, t2 s2: {prettify_radians(theta2_sol2)}')
			self.logger.debug(f'[INKINE] Inverse kinematics solutions in deg: t1 s1: {round(math.degrees(theta1_sol1))}, t2 s1: {round(math.degrees(theta2_sol1))}, t1 s2: {round(math.degrees(theta1_sol2))}, t2 s2: {round(math.degrees(theta2_sol2))}')
			solution_1 = np.array((theta1_sol1, theta2_sol1))
			solution_2 = np.array((theta1_sol2, theta2_sol2))
			return (solution_1, solution_2), True, ''
		except:
			err_info = f'[INKINE] Error arose in analytic inverse kinematics calculations. Traceback: {traceback.print_exc()}'
			self.logger.warning(err_info)
			return None, False, err_info

	def _ikine_analytic(self, point:np.ndarray) -> tuple[np.ndarray, bool, str]:
		"""This function does the inverse kinematics to move the end effector 
		to a specified location in cartesian space. Returns joint solution, success, reason"""
		# check that we can actually move to the position
		x, y = point
		if not self._point_is_in_workspace(point):
			err_info = f'[INKINE] A cartesian command was passed with a point outside of the workspace. X: {x}, Y: {y}'
			self.logger.error(err_info)
			return None, False, err_info

		# the solution 0, 0 has infinite solutions, this is the only case where that happens
		if round(x) == 0 and round(y) == 0:
			theta1, theta2 = self.Q
			if theta2 > 0:
				return np.array((theta1, np.pi)), True, ''
			else:
				return np.array((theta1, -np.pi)), True, ''

		# calculations. Reference: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
		solutions, success, reason = self._helper_ikine_analytic_get_possible_sols(point)
		if not success:
			return None, success, reason
		solution_1, solution_2 = solutions

		# check that the calculated joint angles are valid/in the joint workspace
		solution_1_is_valid = self._joint_pose_is_valid(solution_1)
		solution_2_is_valid = self._joint_pose_is_valid(solution_2)
		
		if solution_1_is_valid and solution_2_is_valid:										# find the solution closest to current position
			self.logger.debug(f'[INKINE] both solutions valid')
			# check to see which is closer from the current position
			distance_1 = self.Q - solution_1
			distance_2 = self.Q - solution_2
			self.logger.debug(f'[IKINE] distance 1: {distance_1}, distance 2: {distance_2}')
			if distance_1.max() < distance_2.max():
				self.logger.debug(f'[INKINE] Solution 1 calculated to be net shorter distance in joint space')
				return np.array(solution_1), True, ''
				# self.set_joint_pose(theta1_sol1, theta2_sol2)		  # this... was strange. I think it was a bug I fixed but should test
			else:																			# distance solution 2 is shorter
				self.logger.debug(f'[INKINE] Solution 2 calculated to be net shorter distance in joint space')
				return np.array(solution_2), True, ''
		elif solution_1_is_valid:															# go to solution 1
			self.logger.debug(f'[INKINE] Solution 1 is only valid solution')
			return np.array(solution_1), True, ''
		elif solution_2_is_valid:															# go to solution 2
			self.logger.debug(f'[INKINE] Solution 2 is only valid solution')
			return np.array(solution_2), True, ''
		else:																				# neither is valid
			self.logger.error(f'[INKINE] Analytic inverse kinematics has failed to solve \
				for joint space solution to x: {x} and y: {y}.')
			return None, False, ''

	def _point_is_in_workspace(self, point:np.ndarray) -> bool:
		max_dist_from_orig = self._link_dh_info[0].a + self._link_dh_info[1].a
		if (point[0]**2 + point[1]**2)**0.5 > max_dist_from_orig:
			return False
		else: return True

	######################################################
	######## Methods for controlling the arm w/ inverse kinematics ########
	######################################################
	def set_cartesian_goal_position(self, X:tuple(float), use_traj:bool=False, traj_time=None) -> bool:
		"""This function does the inverse kinematics to move the end effector 
		to a specified location in cartesian space"""
		# check to make sure this is a valid operation
		if len(X) != 2:
			self.logger.error(f'[SET_CARTESIAN_POSITION] received a vector of incorrect length: {X}')
		elif self.power_status == PowerState.OFF:
			self.logger.warning(f'Cannot execute cartesian movement as motors are off.')
			return

		x, y =  X
		control_mode = self.control_manager.active_controller_name
		self.logger.info(f'Moving robot to cartesian position x: {x} y: {y} using control method {control_mode.name}')
		
		if use_traj:
			# make a trajectory in the cartesian space. This will get referrenced in the main loop, see self._service()
			self._traj = self._produce_cartesian_trajectory(start=self.X, goal=X, traj_time=traj_time)
			return

		if self.control_manager.active_controller.control_space == MathematicalSpaces.CARTESIAN:
			self.control_manager.active_controller.set_target(X)
			self.logger.debug(f'Controller {self.control_manager.active_controller_name.name} has new setpoint: {self.control_manager.active_controller.get_setpoints()}')
			return

		# inverse kinematics for joint space controllers
		# q, success, reason = self._ikine_numerical(np.ndarray(x,y))
		q, success, reason = self._ikine_analytic(np.array([x,y]))	# this intentionally follows the same format as the roboticstoolbox library
		if success:
			self.set_joint_pose(q)		# this will take the correct action based on the active controller
		else:
			self.logger.warning(f'Inverse kinematic solving failed!')
	
	def _produce_cartesian_trajectory(self, start:Collection[float], goal:Collection[float], traj_time=None) -> Trajectory:
		"""Make a time based trajectory from one cartesian position to another.
		
		Trajectory will be converted to the joint space via inverse kinematics"""
		# get a cartesian trajectory
		dist = ((start[0] - goal[0])**2 + (start[1] - goal[1])**2)**0.5
		if traj_time is None:
			total_time = (1/10) * round(10*(dist*6/800 + 2))
		else: total_time= traj_time

		cntrl_spc = self.control_manager.active_controller.control_space
		cartesian_traj:Trajectory = self._produce_general_trajectory(start, goal, total_time, control_space=cntrl_spc, trajectory_space=MathematicalSpaces.CARTESIAN)

		if cntrl_spc == MathematicalSpaces.CARTESIAN:
			return cartesian_traj

		# map the trajectory into the joint space
		qs, qds = [], []
		for x, xd in zip(cartesian_traj.s, cartesian_traj.sd):
			if not self._point_is_in_workspace(x):
				print((x[0]**2 + x[1]**2)**0.5)
				if (x[0]**2 + x[1]**2)**0.5 < 403:
					# this is an odd edge case that causes errors. Essentially a rounding error in forward kinematics 
					x = [i*(400/403.1) for i in x]
			q, success, reason = self._ikine_analytic(x)
			if success: qs.append(q)
			else:
				self.logger.error(f'Inverse kinematics failed during cartesian trajectory generation, cancelling generation. Reason: {reason}')
				return
			
			J_inv, success = self._get_jacob0_inv(q)
			if success:
				qd = np.matmul(J_inv, np.reshape(xd, (2,1)))
				qd = np.reshape(qd, (1, 2))[0]
				qd = np.array([i/2 for i in qd])	# i'm not certain why this is necessary but it works. Probably a math error somewhere.
				qds.append(qd)
			else:
				self.logger.error(f'Inverting jacobian failed, cancelling trajectory generation.')
				return
		
		return Trajectory(
			rtb.tools.trajectory.Trajectory('traj', cartesian_traj.t, np.array(qs), np.array(qds)), 
			control_space=cntrl_spc, 
			trajectory_space=MathematicalSpaces.CARTESIAN
			)



	"""These were a part of the parent class until cartesian controllers got added. Need to validate rtb fkine as well as rtb jcob0 in order
	to move it back. Rtb fkine is used in set_joint_pose"""
	def _helper_update_controls_setpoints_from_trajectory(self) -> NoReturn:
		"""If there is a trajectory, take one step forwards in it, and update controller setpoints with new values.

		Note: when the trajectory is generated, it matches the current servo_rate of the class.
		This library only uses time based trajectories.
		In theory, this means that once every loop the trajectory is used to update the controller.
		In practice the trajectory time vector will likely diverge slightly from what is physcially happening.
		This is deemed acceptable as there are no precision/speed requirements, and the error should be low"""

		if not self.has_trajectory: return

		if self._traj.done:
			self.logger.info(f'Trajectory has been completed!')
			self._traj = None
		else:
			if self._traj.control_space != self.control_manager.active_controller.control_space:
				self.logger.warning(f'Trajectory control space does not match the controller space. Cancelling trajectory.')
				self._traj = None
			else:
				t, s, sd = self._traj.step()
				if self.control_manager.active_controller_name == ControlTypes.ACTUATOR_POSITION_CONTROL:	# SPECIAL CASE
					self.set_joint_pose(s)	# this will write the position to the motors
				# elif self.control_manager.active_controller_name in (ControlTypes.JOINT_PID_TORQUE, ControlTypes.FEED_FORWARDS_VELOCITY_CONTROL):
					# self.control_manager.active_controller.set_target(s, sd)
				
				#### TBD HERE...
				else:
					"""I think this should work"""
					self.control_manager.active_controller.set_target(s, sd)



	def _helper_do_controls(self) -> NoReturn:
		"""Effectively a switch case statement for the control laws"""
		# write values to motors depending on controllers
		if self.control_manager.active_controller_name == ControlTypes.JOINT_PID_TORQUE:
			milli_amps = self.control_manager.active_controller.control_law(self.Q)
			self.set_currents(milli_amps)
		elif self.control_manager.active_controller_name == ControlTypes.FEED_FORWARDS_VELOCITY_CONTROL:
			qd = self.control_manager.active_controller.control_law(self.Q)
			self.logger.debug(f'Setting joint velocity for FEED_FORWARDS_VELOCITY_CONTROL controller {[prettify_radians(i) for i in qd]}')
			self.set_joint_velocities(qd)
		elif self.control_manager.active_controller_name == ControlTypes.CARTESTIAN_TORQUE:
			forces = self.control_manager.active_controller.control_law(self.X)
			forces = np.reshape(forces, (2,1))
			J_t = self._get_jacob0(self.Q).T
			milli_amps = np.matmul(J_t, forces)	# double check the order here, but I matched the way that this happens for cart traj gen
			milli_amps = np.reshape(milli_amps, (1, 2))[0]	# also matched this, untested
			self.set_currents(milli_amps)
		elif self.control_manager.active_controller_name == ControlTypes.RESOLVED_RATE:
			Xd = self.control_manager.active_controller.control_law(self.X)
			J_inv, success = self._get_jacob0_inv(self.Q)
			if success:
				qd = np.matmul(J_inv, np.reshape(Xd, (2,1)))
				qd = np.reshape(qd, (1, 2))[0]
				qd = np.array([i/2 for i in qd])	# i'm not certain why this is necessary but it works. Probably a math error somewhere.
				self.set_joint_velocities(qd)
			else:
				self.logger.warning(f'Jacobian inverse calculations failed during control law calculations!')
	
	async def _service(self):
		"""This task should run continuously."""
		self.logger.info(f'[SERVICE] Begin')
		try:
			while 1:
				# read the motor states. Kinematic model directly interprets from this
				start_time = time.monotonic()
				self.read_motor_states()
				self._helper_update_controls_setpoints_from_trajectory()
				self._helper_do_controls()
				
				elapsed_time = time.monotonic() - start_time
				sleep_time = self._servo_rate - elapsed_time
				if sleep_time < 0:
					self.logger.warning(f'Servo rate is set to faster than the computer can read information from the motors. Elapsed time: {elapsed_time}')
				else:
					await asyncio.sleep(sleep_time)

		except asyncio.CancelledError:
			self.logger.info(f'Service task has ended.') 

	def start_service(self, loop:asyncio.BaseEventLoop=None, new_servo_rate=None):
		self.logger.debug(f'Beginning service loop.')
		if loop is None:
			loop = asyncio.get_event_loop()
		self._service_task = loop.create_task(self._service())
	

	#######################################################
	########### Set routines and demonstrations ###########
	#######################################################
	# def plot(self) -> np.ndarray:
	#   # I dont think I ever got this working, but it was very close.
	# 	# fig = Figure()#figsize=(720, 720), dpi=100)
	# 	# canvas = FigureCanvasAgg(fig)
	# 	# plt = fig.add_subplot(111)
		
	# 	# fig, ax = plt.subplot(1, 1, 1)
	# 	base_location = np.array([0,0])
	# 	j2 = self._joint_2_position
	# 	ee = self.X
	# 	plt.plot([base_location[0], j2[0]],
    #              [base_location[1], j2[1]],
    #              'r-')
	# 	plt.plot([j2[0], ee[0]],
    #              [j2[1], ee[1]],
    #              'r-')
	# 	plt.plot(base_location[0], base_location[1], 'ko')
	# 	plt.plot(j2[0], j2[1], 'ko')
	# 	plt.plot(ee[0], ee[1], 'ko')
	# 	plt.show()