import numpy as np
import pdb
import math
class Resampling:

	"""
	References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
	[Chapter 4.3]
	"""

	def __init__(self):
		"""
		TODO : Initialize resampling process parameters here
		"""

	def multinomial_sampler(self, X_bar):

		"""
		param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
		param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
		"""

		"""
		TODO : Add your code here
		"""

		return X_bar_resampled

	def low_variance_sampler(self, X_bar):
		# Assumes input X_bar is a numpy array of floats

		"""
		param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
		param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
		"""
		X_bar_resampled = []
		X_bar[:,3]=list(map(math.exp,X_bar[:,3]))
		# Normalize weights on (0,1)
		if np.sum(X_bar[:,3]) != 0:
			X_bar[:,3] = X_bar[:,3]/np.sum(X_bar[:,3])
		else:
			print("weights are all zeros")
			X_bar[:,3]=1.0/len(X_bar[:,3])
		M = X_bar.shape[0]
		r = np.random.uniform(0,1/M)
		c = X_bar[0,3]

		i = 0
		for m in range(1, M+1):
			U = r + (m - 1) * (1/M)
			while U > c:
				i += 1
				c += X_bar[i,3]
			X_bar_resampled.append(X_bar[i])
		#for i in range(1,len(X_bar_resampled)):
		#	xPrev=X_bar_resampled[i-1]
		#	x=X_bar_resampled[i]
		#	if abs((xPrev[0]-x[0])**2+(xPrev[1]-x[1])**2+(xPrev[2]-x[2])**2)>0.1:
		#		print("is different")
		return np.array(X_bar_resampled)

if __name__ == "__main__":
	pass
