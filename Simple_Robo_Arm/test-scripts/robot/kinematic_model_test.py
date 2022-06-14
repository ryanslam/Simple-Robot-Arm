#!/usr/bin/env python
"""
@author Bruce Iverson
"""

# external dependencies
from ntpath import join
import roboticstoolbox as rtb
from math import pi
import numpy as np
import matplotlib.pyplot as plt
import math
import spatialmath
# depencies from this package
import sys
sys.path.append(sys.path[0] + '\..')

from robotics.robot import RRTwoLinkKinematicModel


if __name__ == "__main__":
    print(f'DEMONSTRATION AND TESTING OF SIMPLE ROBOT ARM SIMULATION!')
    kinematic_model = RRTwoLinkKinematicModel()
    print(kinematic_model)        # displays a prettified kinematic_model arm

    print(f'\nFORWARD KINEMATICS!')
    joints = np.array([0, 0])  # joint positions in radians 
    print(f'Joints: {joints}, type: {type(joints)}')
    T = kinematic_model.fkine(joints)
    print(f'x: {T.t[0]}, y: {T.t[1]}')

    print(f'\nINVERSE KINEMATICS')
    for x, y in ((1600, 0), (800, 800), (0, 1600)):
        T = spatialmath.pose3d.SE3(x, y, 0)
        sol = kinematic_model.ikine_LM(T)     # named tuple?
        print(f'ikine_lm soution for x: {x}, y: {y} case: \n{sol}')
        # q = sol['q']
        q_sol, success, reason = sol[0:3]
        print(f'Degrees: {[round(math.degrees(i)) for i in q_sol]}')
    
    print('')
    print(f'Jacobe: \n {kinematic_model.jacobe(q_sol)}')
    print(f'Jacob0: \n {kinematic_model.jacob0(q_sol)}')

    # joints = [0, 0]                  # points towards positive x
    # kinematic_model.plot(joints, block=True)      # block keeps it open
    # joints = [pi/2, 0]               # this one points towards positive Y, straight line
    # kinematic_model.plot(joints, block=True)      # block keeps it open
    # joints = [0, pi/2]               # also points towrads positive y
    # kinematic_model.plot(joints, block=True)      # block keeps it open
    # joints = [pi/2, pi/2]               # also points towrads positive y
    # kinematic_model.plot(joints, block=True)      # block keeps it open

    # joints = [0, 0, 0]                  # points towards positive x
    # kinematic_model.plot(joints, block=True)      # block keeps it open
    # joints = [pi/2, 0, 0]               # this one points towards positive Y, straight line
    # kinematic_model.plot(joints, block=True)      # block keeps it open
    # joints = [0, pi/2, 0]               # also points towrads positive y
    # kinematic_model.plot(joints, block=True)      # block keeps it open
    # joints = [0, 0, pi/2]               # positive x and then positive Y, bent joint
    # kinematic_model.plot(joints, block=True)      # block keeps it open
    # joints = [pi/2, pi/2, 0]            # these appear to sum... points in negative x
    # kinematic_model.plot(joints, block=True)      # block keeps it open
    # # joints = [pi/2, pi/2, 0]            # these appear to sum... points in negative x
    # # kinematic_model.plot(joints, block=True)      # block keeps it open

    print(f'Commence noodle kinematic_model trajectory simulation')
    start = kinematic_model.fkine([0, 0])    # SE3 transform
    end = kinematic_model.fkine([pi/2, pi/2])
    print(f'Start: \n {start} \n end: \n {end}')
    traj = kinematic_model.jtraj(
        start,
        end,
        100
    )
    kinematic_model.plot(traj.q, dt=.025, name=True, eeframe=True, jointaxes=True)