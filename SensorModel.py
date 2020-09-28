import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb
from MapReader import MapReader
from rayTrace import rayTrace

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        self.map=occupancy_map
        #note: adjust these later
        self.stdDevHit=1
        self.lambdaShort=0.01
        self.measureMax=8191
        self.zHit=0.25
        self.zShort=0.25
        self.zMax=0.25
        self.zRand=0.25

    def pHit(self,zkt,xt,xtLaser):
        if 0<=zkt and zkt<=self.measureMax:
            zktStar = rayTrace(xtLaser,self.map)
            cdf = norm.cdf(zkt, loc=zktStar, scale=self.stdDevHit)
            # if cdf is zero, zkt is far from zktStar, probability is ~0
            if cdf == 0:
                return 0
            normalizer = 1/cdf
            return normalizer * norm.pdf(zkt, loc=zktStar, scale=self.stdDevHit)
        return 0

    def pShort(self,zkt,xt,xtLaser):
        zktStar=rayTrace(xtLaser,self.map)
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
        xRobot=z_t1_arr[1]
        yRobot=z_t1_arr[2]
        thRobot=z_t1_arr[3]
        xLaser=z_t1_arr[4]
        yLaser=z_t1_arr[5]
        thLaser=z_t1_arr[6]
        q=1

        for k in range(7,187):
            #compute zkt
            zkt=z_t1_arr[k]
            xt=[xRobot,yRobot,thRobot-math.pi/2+(k-7)*math.pi/180]
            xtLaser = [xLaser,yLaser,thLaser-math.pi/2+(k-7)*math.pi/180]
            p=self.zHit*self.pHit(zkt,xt,xtLaser)\
            +self.zShort*self.pShort(zkt,xt,xtLaser)\
            +self.zMax*self.pMax(zkt)\
            +self.zRand*self.pRand(zkt)
            p=p*q

        return q

if __name__=='__main__':
    pass
