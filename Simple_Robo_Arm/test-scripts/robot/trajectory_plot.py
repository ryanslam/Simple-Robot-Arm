import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt


class RRTwoLinkKinematicModel(rtb.DHRobot):
	"""A wrapped class for a Denavit-Hatenberg defined robot using Peter Corkes robotics toolbox.
	
	This conforms to conventions of the roboticstoolbox as well as providing some additional functions"""

	def __init__(self, name:str='rr_two_link'): 
		# initialize the parent class with model dh parameters. This model kinematic with the addition of inertial parameters. Friction is ignored.
		"""
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
		L = [
			rtb.RevoluteDH(d=0, alpha=0, a=200, m=10, qlim=np.array([-np.pi, np.pi]), name='link1'),	# transform to link 1
			rtb.RevoluteDH(d=0, alpha=0, a=200, m=10, qlim=np.array([-np.pi, np.pi]), name='link2'),	# transform to second motor
        ]
		
		super().__init__(L, name=name)
		self.addconfiguration('q0', [0,0], unit='rad')


start = [0]
goal = [400]
total_time = 5	#seconds
n_steps = int(5*total_time)
t = np.linspace(0, total_time, num=n_steps)
traj = rtb.tools.trajectory.jtraj(start, goal, t)
fig, ax = plt.subplots()
print(t)
print(traj.q[:, 0])
print(traj.q)
# plt.plot(t, traj.q[:,0], color='r')
ax.plot(t, traj.q[:,0], 'ro')
ax.plot(t, traj.qd[:,0], 'bo')
ax.plot(t, traj.qdd[:,0], 'go')
plt.show()


traj = rtb.tools.trajectory.jtraj(start, goal, t)
fig, ax = plt.subplots()
# plt.plot(t, traj.q[:,0], color='r')
ax.plot(t, traj.q[:,0], 'ro')
ax.plot(t, traj.qd[:,0], 'bo')
ax.plot(t, traj.qdd[:,0], 'go')
plt.show()

import spatialmath
kinematic_model = RRTwoLinkKinematicModel('rr_two_link')
print(f'Commence noodle kinematic_model trajectory simulation')

start = kinematic_model.fkine([0, 0])    # SE3 transform
end = kinematic_model.fkine([np.pi/2, np.pi/2])

print(f'Start: \n {start} \n end: \n {end}')

traj = kinematic_model.jtraj(
    start,
    end,
    100
)
dt = t[1] - t[0]
print(f'dt is: {dt}')
kinematic_model.plot(traj.q, dt=dt, name=True, eeframe=True, jointaxes=True)
