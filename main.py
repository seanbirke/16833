import numpy as np
import sys
import pdb
from multiprocessing import Pool
from MapReader import MapReader
from MotionModel import MotionModel
from SensorModel import SensorModel
from Resampling import Resampling

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time

def visualize_map(occupancy_map):
	fig = plt.figure()
	# plt.switch_backend('TkAgg')
	mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
	plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def visualize_timestep(X_bar, tstep):
	x_locs = X_bar[:,0]/10.0
	y_locs = X_bar[:,1]/10.0
	scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
	plt.pause(0.00001)
	scat.remove()

def init_particles_random(num_particles, occupancy_map):

	# initialize [x, y, theta] positions in world_frame for all particles
	y0_vals = np.random.uniform( 3500, 4500, (num_particles, 1) )
	x0_vals = np.random.uniform( 4000, 4700, (num_particles, 1) )
	theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

	# initialize weights for all particles
	w0_vals = np.ones( (num_particles,1), dtype=np.float64)
	#w0_vals = w0_vals / num_particles
	w0_vals_norm=[w/num_particles for w in w0_vals]
	X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals_norm))

	return X_bar_init

def init_particles_freespace(num_particles, occupancy_map):

	# initialize [x, y, theta] positions in world_frame for all particles

	"""
	TODO : Add your code here
	"""

	return X_bar_init

def main():
	"""
	Description of variables used
	u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
	u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
	x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
	x_t1 : particle state belief [x, y, theta] at time t [world_frame]
	X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
	z_t : array of 180 range measurements for each laser scan
	"""

	"""
	Initialize Parameters
	"""
	src_path_map = '../data/map/wean.dat'
	src_path_log = '../data/log/robotdata1.log'

	map_obj = MapReader(src_path_map)
	occupancy_map = map_obj.get_map()
	logfile = open(src_path_log, 'r')

	motion_model = MotionModel()
	sensor_model = SensorModel(occupancy_map)
	resampler = Resampling()

	num_particles = 100
	X_bar = init_particles_random(num_particles, occupancy_map)

	vis_flag = 1

	"""
	Monte Carlo Localization Algorithm : Main Loop
	"""
	if vis_flag:
		visualize_map(occupancy_map)

	first_time_idx = True
	#initialize worker threads
	p=Pool(15)
	for time_idx, line in enumerate(logfile):
		# Read a single 'line' from the log file (can be either odometry or laser measurement)
		meas_type = line[0] # L : laser scan measurement, O : odometry measurement
		meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double
		odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
		time_stamp = meas_vals[-1]

		# if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure odometry measurements for now (faster debugging)
			# continue

		if (meas_type == "L"):
			 odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
			 ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan

		print("Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s")

		if (first_time_idx):
			u_t0 = odometry_robot
			first_time_idx = False
			continue

		X_bar_new = np.zeros( (num_particles,4), dtype=np.float64)
		u_t1 = odometry_robot
		# X_bar.shape[0] (length) decreases from 500 to 499 after time step 1

		#PARALLELIZED MOTION MODEL

		x_t0Arr=[[u_t0,u_t1,X_bar[m,0:3]] for m in range(0,X_bar.shape[0])]
		x_t1Arr=p.map(motion_model.par_update,x_t0Arr)

		#PARALLELIZED SENSOR MODEL

		if (meas_type=="L"):
			z_t=ranges
			sensInArr=[[z_t,x_t1Arr[m]] for m in range(0,X_bar.shape[0])]
			w_tArr=p.map(sensor_model.par_beam_range_finder_model,sensInArr)
			for m in range(0,X_bar.shape[0]):
				X_bar_new[m,:]=np.hstack((x_t1Arr[m],w_tArr[m]))	
		else:
			for m in range(0,X_bar.shape[0]):
				X_bar_new[m,:]=np.hstack((x_t1Arr[m],X_bar[m,3]))	
#		for m in range(0, X_bar.shape[0]):	
#			
#			#MOTION MODEL
#			
#			x_t0 = X_bar[m, 0:3]
#			x_t1 = motion_model.update(u_t0, u_t1, x_t0)
#
#			
#			#SENSOR MODEL
#			
#			if (meas_type == "L"):
#				z_t = ranges
#				w_t = sensor_model.beam_range_finder_model(z_t, x_t1)
#				# w_t = 1/num_particles
#				X_bar_new[m,:] = np.hstack((x_t1, w_t))
#			else:
#				X_bar_new[m,:] = np.hstack((x_t1, X_bar[m,3]))
		X_bar = X_bar_new
		u_t0 = u_t1

		"""
		RESAMPLING
		"""
		X_bar = resampler.low_variance_sampler(X_bar)

		if vis_flag:
			visualize_timestep(X_bar, time_idx)

if __name__=="__main__":
	main()
