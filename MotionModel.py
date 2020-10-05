import sys
import numpy as np
import math

class MotionModel:

	"""
	References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
	[Chapter 5]
	"""

	def __init__(self):

		# rot_1
		self.alpha_1 = 0.1
		# trans
		self.alpha_2 = 10
		# trans
		self.alpha_3 = 10
		# rot_1 + rot_2
		self.alpha_4 = 10


	def par_update(self,l):
		return self.update(l[0],l[1],l[2])	
	def update(self, u_t0, u_t1, x_t0):
		"""
		param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
		param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
		param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
		param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]

		"""

		d_rot1 = math.atan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2]
		d_trans = math.sqrt((u_t0[0] - u_t1[0])**2 + (u_t0[1] - u_t1[1])**2)
		d_rot2 = u_t1[2] - u_t0[2] - d_rot1

		dh_rot1 = d_rot1 - np.random.normal(scale = (self.alpha_1 * d_rot1**2 + self.alpha_2 * d_trans**2))
		dh_trans = d_trans - np.random.normal(scale = (self.alpha_3 * d_trans**2 + self.alpha_4 * (d_rot1**2 + d_rot2**2)))
		dh_rot2 = d_rot2 - np.random.normal(scale = (self.alpha_1 * d_rot2**2 + self.alpha_2 * d_trans**2))

		x_p = x_t0[0] + dh_trans * math.cos(x_t0[2] + dh_rot1)
		y_p = x_t0[1] + dh_trans * math.sin(x_t0[2] + dh_rot1)
		theta_p = x_t0[2] + dh_rot1 + dh_rot2

		x_t1 = [x_p, y_p, theta_p]

		return x_t1

if __name__== "__main__":
	pass
