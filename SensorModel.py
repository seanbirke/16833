import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb
from MapReader import MapReader
from rayTrace import rayTrace
#import multiprocessing
#from multiprocessing.pool import ThreadPool
#adjusts a laser distance reading to the distance fom robot to reading target
def adjuster(reading,adj,angle):
	return math.sqrt(reading**2 + adj**2-2*reading*adj*math.cos(math.pi-angle))
#same as adjuster but takes in a list
def adjusterL(l):
	return adjuster(l[0],l[1],l[2])
#takes in list, returns log sum of list
def logsum(l):
	return sum(map(math.log,l))
class SensorModel:

	"""
	References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
	[Chapter 6.3]
	"""

	def __init__(self, occupancy_map):

		self.oMap=occupancy_map
		#note: adjust these later
		self.stdDevHit=100
		self.lambdaShort=0.1
		self.measureMax=8183
		self.zHit=0.4
		self.zShort=0.4
		self.zMax=0.1
		self.zRand=0.1
		#certainty defines threshold for assuming a grid is occupied
		self.certainty=0.9
		#laserSubsample defines laser sampling; every nth laser is sampled
		self.laserSubsample=5

	def pHit(self,zkt,zktStar):
		if 0<=zkt and zkt<=self.measureMax:
			cdf = norm.cdf(zkt, loc=zktStar, scale=self.stdDevHit)
			# if cdf is zero, zkt is far from zktStar, probability is ~0
			if cdf < 0.0001:
				return 0
			normalizer = 1/cdf
			return normalizer * norm.pdf(zkt, loc=zktStar, scale=self.stdDevHit)
		return 0

	def pShort(self,zkt,zktStar):
		if 0<=zkt and zkt<=zktStar:
			n=1.0/(1-math.exp(-self.lambdaShort*zktStar))
			return n*self.lambdaShort*math.exp(-self.lambdaShort*zkt)
		return 0

	def pMax(self,zkt):
		if zkt>=self.measureMax:
			return 1
		return 0

	def pRand(self,zkt):
		if 0<=zkt and zkt < self.measureMax:
			return 1.0/self.measureMax
		return 0
	#calculates p
	def calcP(self,zs):
		zkt=zs[0]
		zktStar=zs[1]
		return self.zHit*self.pHit(zkt,zktStar)\
			+self.zShort*self.pShort(zkt,zktStar)\
			+self.zMax*self.pMax(zkt)\
			+self.zRand*self.pRand(zkt)
	#same as beam_range_finder_model but takes in a list imput
	def par_beam_range_finder_model(self,l):
		return self.beam_range_finder_model(l[0],l[1])
	def beam_range_finder_model(self, z_t1_arr, x_t1):
		"""
		param[in] z_t1_arr : laser range readings [array of 180 values] at time t
		param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
		param[out] prob_zt1 : likelihood of a range scan zt1 at time t
		"""
		#initial check to see if x_t1 is in a wall; if so then we don't
		#need to run rayTrace.
		inWall=False
		if self.oMap[int(x_t1[1]/10)][int(x_t1[0]/10)]>=self.certainty:
			inWall=True
		#q is for now represented by  ln(q), so q=e
		q=0
		#want: q=log(p1)+log(p2)+log(p3)+log(p4)
		#spawn 180 workers
		#print("pooling")
		#poo=ThreadPool(10)
		#concatenate all inputs into a single list
		#print("staring")
		#pointRays=[[x_t1,(-math.pi/2+k*math.pi/180),
		#			self.oMap,self.certainty] for k in range(0,180,self.laserSubsample)]
		#zktRays=[ [z_t1_arr[k],25,
		#			abs(-math.pi/2+k*math.pi/180)]for k in range(0,180,self.laserSubsample)]
		#print("mapping")
		#perform ray tracing on our position
		#zktStarArr=poo.map(rayTrace,pointRays)
		#adjust laser readings to more accurate distance to robot
		#zktArr=poo.map(adjusterL,zktRays)
		#zippedZs=[ [zktArr[k],zktStarArr[k]] for k in range(len(zktArr))]
		#calculate values of p, then perform logsum
		#q=logsum(poo.map(self.calcP,zippedZs))
		#print("done parallelizing")
		for k in range(0,180,self.laserSubsample):

			#same position but changed for laser
			lasAngle=-math.pi/2+k*math.pi/180
			#compute zkt; adjust for laser position
			zkt=adjuster(z_t1_arr[k],25,abs(lasAngle))
			#compute zktStar estimate using ray tracing
			zktStar=25
			if not inWall:
				zktStar=rayTrace([x_t1,lasAngle,self.oMap,self.certainty])
			p=self.zHit*self.pHit(zkt,zktStar)\
			+self.zShort*self.pShort(zkt,zktStar)\
			+self.zMax*self.pMax(zkt)\
			+self.zRand*self.pRand(zkt)
			#q=log(p1)+log(p2)+..
			if p!=0:
				q=q+math.log(p)
			else:
				return 0
		#print(math.exp(q))
		print(x_t1)
		return math.exp(q)

if __name__=='__main__':
	pass
