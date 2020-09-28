import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb
from MapReader import MapReader
from rayTrace import rayTrace

def adjuster(reading,adj,angle):
    return math.sqrt(reading**2 + adj**2-2*reading*adj*math.cos(angle))

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        self.oMap=occupancy_map
        #note: adjust these later
        self.stdDevHit=1
        self.lambdaShort=0.01
        self.measureMax=8191
        self.zHit=0.7
        self.zShort=0.2
        self.zMax=0.05
        self.zRand=0.05
        self.certainty=0.9

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
        if zkt==self.measureMax:
            return 1
        return 0

    def pRand(self,zkt):
        if 0<=zkt and zkt < self.measureMax:
            return 1.0/self.measureMax
        return 0

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
        q=1

        for k in range(1,180):

            #same position but changed for laser
            lasAngle=-math.pi/2+k*math.pi/180
            #compute zkt; adjust for laser position
            zkt=adjuster(z_t1_arr[k],25,abs(lasAngle))
            #compute zktStar estimate using ray tracing
            zktStar=0
            if not inWall:
                zktStar=rayTrace(x_t1,lasAngle,self.oMap,self.certainty)

            p=self.zHit*self.pHit(zkt,zktStar)\
            +self.zShort*self.pShort(zkt,zktStar)\
            +self.zMax*self.pMax(zkt)\
            +self.zRand*self.pRand(zkt)
            q=q*p

        return q

if __name__=='__main__':
    pass
