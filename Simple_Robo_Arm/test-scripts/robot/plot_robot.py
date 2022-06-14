from __future__ import annotations
import sys
import logging
import math
import time
import asyncio
import matplotlib.pyplot as plt
import numpy as np

sys.path.append(sys.path[0] + '/..')
print(sys.path)
import robotics.robot as robot

if __name__ == "__main__":
    rr_two_link = robot.RRTwoLink()
    rr_two_link.plot()
    # img:np.ndarray = rr_two_link.plot()
    # plt.plot(img)
    # plt.show()
