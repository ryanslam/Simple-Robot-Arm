#!/usr/bin/env python
"""
@author Bruce Iverson
"""

from __future__ import annotations
from typing import Any, Collection, NoReturn, Optional, TypeVar, Generic

import simple_pid
from enum import IntEnum, Enum
import numpy as np
import logging
# import roboticstoolbox as rtb

from .dynamixel_wrapper.wrapper import DynamixelControlMode
from rsl.rr_two_link.util import get_simple_logger


# some Types to help with type hinting. Some of these are used/referred to in the robot.py file too.
Gains = Collection[float]       # PIDs
PIDs = Collection[simple_pid.PID]     # the convention for these will be to order them 
Shape = TypeVar("Shape")
DType = TypeVar("DType")


# this is not ideal, but I wanted to convey the shapes of the vectors/matrixes, and shape type hinting is not yet supported with numpy 
class Array(np.ndarray, Generic[Shape, DType]):
    """Use this to type-annotate numpy arrays e.g.
        image: Array['H,W,3', np.uint8]
        xy_points: Array['N,2', float]
        nd_mask: Array['...', bool]"""


# for both joints and for cartesian positions, two types are defined, one with shape (1,2) that makes indexing the array easy,
# and one with shape (2,1) that is necessary for linear algebra operations
JointPosition = Array['1,2', float]
JointPositionVector = Array['2,1', float]
Cartesian2DPosition = Array['1,2', float]
Cartesian2DPositionVector = Array['2,1', float]


class MathematicalSpaces(Enum):
	JOINT = "JOINT_SPACE"
	CARTESIAN = "CARTESIAN"


class ControlTypes(IntEnum):
    ACTUATOR_POSITION_CONTROL = 0
    # ACTUATOR_EXTENDED_POSITION_CONTROL = 1
    FEED_FORWARDS_VELOCITY_CONTROL = 2
    RESOLVED_RATE = 3
    JOINT_PID_TORQUE = 4
    CARTESTIAN_TORQUE = 5

    @classmethod
    def get_actuator_mode(cls, mode:ControlTypes) -> DynamixelControlMode:
        actuator_control_mode_from_control_type:dict[ControlTypes, DynamixelControlMode] = {
            ControlTypes.ACTUATOR_POSITION_CONTROL: DynamixelControlMode.POSITION,
            ControlTypes.FEED_FORWARDS_VELOCITY_CONTROL: DynamixelControlMode.VELOCITY,
            ControlTypes.JOINT_PID_TORQUE: DynamixelControlMode.CURRENT,     # may need to change this to pwm...
            ControlTypes.RESOLVED_RATE: DynamixelControlMode.VELOCITY,
            ControlTypes.CARTESTIAN_TORQUE: DynamixelControlMode.CURRENT  # may need to change this to pwm...
        }
        return actuator_control_mode_from_control_type[mode]


class ControllerManager:
    def __init__(self):
        self.kps = [55, 55]
        self.kis = [0, 0]
        self.kds = [7, 4]
        self.controllers: dict[ControlTypes, PIDController] = {
            ControlTypes.ACTUATOR_POSITION_CONTROL: PIDController(ControlTypes.ACTUATOR_POSITION_CONTROL, MathematicalSpaces.JOINT), # a dummy placeholder
            # ControlTypes.ACTUATOR_EXTENDED_POSITION_CONTROL: PIDController(ControlTypes.ACTUATOR_POSITION_CONTROL, MathematicalSpaces.JOINT), # a dummy placeholder
            ControlTypes.FEED_FORWARDS_VELOCITY_CONTROL: FeedForwardsVelocityController(),
            # ControlTypes.JOINT_PID_TORQUE: JointPositionPID(),
            ControlTypes.JOINT_PID_TORQUE: JointPositionPID(kps=self.kps, kis=self.kis, kds=kself.ds),
            ControlTypes.RESOLVED_RATE: ResolvedRate(),                             
            ControlTypes.CARTESTIAN_TORQUE: CartesianTorque(),
            }
        
        # below is essentially just used as a key for the above dict (which should be static)
        self.active_controller_name:ControlTypes = ControlTypes.ACTUATOR_POSITION_CONTROL
        # NOTETOSELF: this should happen in the robot (only change controllers when not moving. PIDController doesnt really have the contex, I think. Also, issues with )
        # self.is_moving = True   # this is used for safety checks, initialize as unsafe

    @property
    def active_controller(self) -> PIDController:
        return self.controllers[self.active_controller_name]

    def set_active_controller(self, new_controller: ControlTypes) -> bool:
        if new_controller in self.controllers.keys():# and not self.is_moving:       # check that this is known
            self.active_controller_name = new_controller
            return True
        else:
            print(f'Error! in controller manager')
            return False
    
    # Set the active controller gains.
    def set_active_controller_gains(self, new_controller: ControlTypes, kps, kis, kds):
        if new_controller in self.controllers.keys() and new_controller != ControlTypes.ACTUATOR_POSITION_CONTROL:
            self.kps = kps
            self.kis = kis
            self.kds = kds
            return True
        else:
            print(f'Error! Unable to set gains.')
            return False


class PIDController:
    """Parent class for PID controllers. Essentially a wrapper for supporting multiple PIDs of the simple_pid.PID type.
    Some of the methods here usually get overwritten in the child classes, they exist here to help with type hinting. Additionally, this class has a few small utility items
    
    This class should probably get split out into parent Controller class and a child PIDController Class, but for now all of the controllers 
    use PIDs so..."""

    def __init__(self, name:ControlTypes, control_space):
        self.name = name
        # self.logger = get_simple_logger(__name__ + '.' + name.name.lower(), verbosity=logging.INFO)
        self.logger = get_simple_logger(name.name.lower(), verbosity=logging.INFO)
        self.setpoint_vels = np.array([0, 0])  # joint velocities or cartesian velocities [x y]
        self.control_space = control_space
        self._pids:PIDs = None   # this gets initialized by child classes

    def _produce_pid_controllers(self, n_pids, kps:Gains, kis:Gains=None, kds:Gains=None, initial_setpoints:Collection[float]=None) -> PIDs:
        # default for initial setpoints is 0
        for item in (kis, kds, initial_setpoints):
            if item is None:
                item = [0 for _ in range(n_pids)]

        # ensure valid parameters were passed
        for item in (kps, kis, kds, initial_setpoints):
            if len(item) != n_pids:
                print(f'Error creating PIDs for {self.name.name}! In particular, item with values {item} was passed.')
                raise ValueError

        return [simple_pid.PID(kps[i], kis[i], kds[i], initial_setpoints[i]) for i in range(n_pids)]

    def _set_pid_saturation_points(self, limits:Collection[float]) -> NoReturn:
        for i, pid in enumerate(self._pids):
            # pid.sample_time=0.001 # this limits how often the pid can get updated
            lim = limits[i]
            pid.output_limits = (-lim, lim)

    @property
    def set_target(self):
        """Returns the correct method for setting target based on the control type"""
        if self.control_space == MathematicalSpaces.JOINT:
            return self.set_joint_target
        else:
            return self.set_cartesian_target

    def set_joint_target(self, q_goal:JointPosition, qd_goal:JointPosition=np.array([0, 0])) -> NoReturn:
        if self._pids is None:
            print(f'controller setjoint target failed as pids are not initialized!')
            return

        assert len(self._pids) == len(q_goal)

        for pid, targ in zip(self._pids, q_goal):
            pid.setpoint=targ
        self.setpoint_vels = qd_goal    # this is not always used

    def set_cartesian_target(self, X_goal:Cartesian2DPosition, Xd_goal:Cartesian2DPosition=np.array([0, 0])) -> NoReturn:
        for pid, targ in zip(self._pids, X_goal):
            pid.setpoint=targ
            print(f'pid setpoint updated to: {targ}')
        self.setpoint_vels = Xd_goal

    def get_gains(self) -> list[Gains]:
        """Useful for telemetry sent from server to client"""
        return [pid.tunings for pid in self._pids]

    def set_gains_for_one_pid(self, pid_index, kp, ki, kd) -> bool:
        """Sets controllers with new gains. Returns success"""
        if True:    # left structure to check valid gains here
            self._pids[pid_index].tunings = (kp, ki, kd)
            self.logger.info(f'PID gains for link {pid_index} have been updated to: {self._pids[pid_index].tunings}')
            return True
        else:
            return False

    # this was unused
    def get_setpoints(self) -> np.ndarray:
        """Returns a vector of the current setpoints."""
        if self._pids is None: return None
        return np.array([pid.setpoint for pid in self._pids])


class JointPIDController(PIDController):
    """A controller that has a PID associated with each joint of the robot. This means that setpoints, inputs, etc are 
    vectors with shape [1, N] where N is the number of joints."""

    def __init__(self, name:ControlTypes):
        super().__init__(name, control_space=MathematicalSpaces.JOINT)

    def _calculate_pid_outputs(self, q:JointPosition) -> np.ndarray:
        """Updates the _pids for each link, and returns what the output joint torques should be."""
        output_list = [pid(theta) for pid, theta in zip(self._pids, q)]
        return np.array(output_list)


class JointPositionPID(JointPIDController):
    def __init__(self, kps=[55, 35], kis=[0,0], kds=[7,4]):
        super().__init__(ControlTypes.JOINT_PID_TORQUE)
        # self.control_type: ControlTypes = ControlTypes.ACTUATOR_POSITION_CONTROL
        
        # Original values for kps, kis, and kds.
        # kps = [55, 35]
        # kis = [0, 0]
        # kds = [7, 4]

        # Allows for reinitilization of controller gains.
        self.kps = kps
        self.kis = kis
        self.kds = kds

        self._pids = self._produce_pid_controllers(2, self.kps, self.kis, self.kds, (0, 0))
        saturation_point = 1000 # milli amps
        self._set_pid_saturation_points([saturation_point, saturation_point])

    def control_law(self, q:JointPosition) -> np.ndarray:
        return self._calculate_pid_outputs(q)


class FeedForwardsVelocityController(JointPIDController):

    def __init__(self, n_links=2):
        super().__init__(ControlTypes.FEED_FORWARDS_VELOCITY_CONTROL)
        
        kps = [0.5, 0.4]
        kis = [0, 0]
        kds = [0, 0]
        
        self._pids = self._produce_pid_controllers(2, kps, kis, kds, (0, 0))
        max_joint_velocity = np.pi
        self._set_pid_saturation_points([max_joint_velocity, max_joint_velocity])
    
    def control_law(self, q:JointPosition) -> np.ndarray:
        """Takes in current joint position and returns the speed the joints should be moving at in rad/s"""
        
        target_speeds = self.setpoint_vels + self._calculate_pid_outputs(q)
        # need to include a warning here if it gets saturated?
        return target_speeds


class CartesianPIDController2D(PIDController):
    """A controller that has a PID associated the 2D end effector positional error, one PID for the 
    x direction and one with the y direction, both having the same gains.
    
    Setpoints, inputs, etc are np.ndarray with shape [1, 2]."""

    def __init__(self, name:ControlTypes):
        super().__init__(name, control_space=MathematicalSpaces.CARTESIAN)

    def set_gains(self, kps, kis, kds) -> NoReturn:
        for item in kps, kis, kds:
            if len(item) != len(self._pids):
                raise ValueError
                
        for i, pid in enumerate(self._pids):
            self.set_gains_for_one_pid(i, kps[i], kis[i], kds[i])

    def _calculate_pid_outputs(self, X:Cartesian2DPosition) -> np.ndarray:
        """Updates the _pids for each link, and returns what the output joint torques should be."""
        output_list = [pid(x) for pid, x in zip(self._pids, X)]
        return np.array(output_list)
 

class ResolvedRate(CartesianPIDController2D):

    def __init__(self):
        super().__init__(ControlTypes.RESOLVED_RATE)
        kps, kis, kds = [0.7], [0], [0]
        kps, kis, kds = [item*2 for item in (kps, kis, kds)]

        self._pids = self._produce_pid_controllers(2, kps, kis, kds, (400, 0))
        max_vel = 600      # mm/s
        self._set_pid_saturation_points([max_vel, max_vel])

    def control_law(self, X:Cartesian2DPosition) -> np.ndarray:
        """Takes in current joint position and returns the speed in cartesian space the end effector should be moving at in mm/s"""
        
        cartesian_velocity = self._calculate_pid_outputs(X)
        print(f'ouput: {cartesian_velocity}')
        # need to include a warning here if it gets saturated?
        return cartesian_velocity


class CartesianTorque(CartesianPIDController2D):

    def __init__(self):
        super().__init__(ControlTypes.CARTESTIAN_TORQUE)

        kps, kis, kds = [0.005], [0], [0]
        kps, kis, kds = [item*2 for item in (kps, kis, kds)]
        self._pids = self._produce_pid_controllers(2, kps, kis, kds, (400, 0))

        max_current = 1000      # milli amps
        self._set_pid_saturation_points([max_current, max_current])
   
    def control_law(self, X:Cartesian2DPosition) -> np.ndarray:
        """Takes in current cartesian position velocity and returns the Force with which the end effector should push"""
        force_vec = self._calculate_pid_outputs(X)
        return force_vec