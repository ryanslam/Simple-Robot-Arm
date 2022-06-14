#!/usr/bin/env python
"""
@author Bruce Iverson
"""

from __future__ import annotations
import sys
import logging
from math import pi
import numpy as np
import sys

sys.path.append(sys.path[0] + '/../..')
from rsl.rr_two_link.robotics.robot import *
from rsl.rr_two_link.util import prettify_radians

arm = RRTwoLink(log_level=logging.INFO)

start = (0, 0)
way_points = [start, (pi/2, pi/2)]

for pose in way_points:
    print(f'\n \nQ: {[prettify_radians(q) for q in pose]}')
    print(f'J: \n{arm._get_jacob0(pose)},')
    J_inv, success = arm._get_jacob0_inv(pose)
    print(f' Jinv \n{J_inv}')
    xd = np.array([-1, 0])
    print(f'xd: {xd}')
    print(f'qd: {np.matmul(J_inv, np.reshape(xd, (2,1)))}')

loop = asyncio.get_event_loop()
loop.run_until_complete(arm.shutdown())

