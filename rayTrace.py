import math
#note: assumes map pixels are 10 cm
#estimates distance to nearest surface by following traj of x until hitting surface in map
#pos=[x,y,theta] of the robot,lAngle=angle offset of laser
#also adjusts for the ~25cm offset from the actual robot position
def adjuster(reading,adj,angle):
    return math.sqrt(reading**2 + adj**2-2*reading*adj*math.cos(angle))
def rayTrace(pos,lAngle,map):
    #find current block in map
    adj=25
    laserTheta=pos[2]+lAngle
    xSlope=math.cos(laserTheta)
    ySlope=math.sin(laserTheta)
    res=10
    #initialize rayPos
    rayPos=[pos[0],pos[1],laserTheta]
    maxX=map.shape[0]*10
    maxY=map.shape[1]*10
    wallPosX=(maxX-pos[0])
    wallPosY=(maxY-pos[1])
    eps=1
    withinX = rayPos[0] > 0 and rayPos[0] < maxX
    withinY = rayPos[1] > 0 and rayPos[1] < maxY
    notWall = (withinX and withinY and map[int(rayPos[1] / 10)][int(rayPos[0] / 10)] < 0)
    while(withinX and withinY and notWall):
        #increment rayPos by eps along its direction

        #update rayPos
        rayPos=[rayPos[0]+xSlope*eps,rayPos[1]+iySlope*eps,rayPos[2]]
        withinX=rayPos[0]>0 and rayPos[0]<maxX
        withinY=rayPos[1]>0 and rayPos[1]<maxY
        notWall = (withinX and withinY and
                map[int(rayPos[1]/10)][int(rayPos[0]/10)] < 0)

    #return ray length, with 25cm adjustment for laser
    readingLen=math.sqrt((pos[0]-rayPos[0])**2+(pos[1]-rayPos[1])**2)
    finalLen=adjuster(readingLen,adj,abs(lAngle))
    #print(finalLen,withinX,withinY,notWall)
    return finalLen
