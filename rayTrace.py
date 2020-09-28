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
    withinX = rayPos[0] > 0 and rayPos[0] < maxX
    withinY = rayPos[1] > 0 and rayPos[1] < maxY
    notWall = (withinX and withinY and map[int(rayPos[1] / 10)][int(rayPos[0] / 10)] < 0)
    while(withinX and withinY and notWall):
        toNext=0
        toNextX=0
        toNextY=0
        eps=0.00001
        if(xSlope<eps):
            toNext=max((rayPos[1]%10)/ySlope , (10 -rayPos[1]%10)/ySlope)
        else:
            toNextX=max((rayPos[0]%10)/xSlope , (10 -rayPos[0]%10)/xSlope)
        if(ySlope<eps):
            toNext=max((rayPos[0]%10)/xSlope , (10 -rayPos[0]%10)/xSlope)
        else:
            toNextY=max((rayPos[1]%10)/ySlope , (10 -rayPos[1]%10)/ySlope)
        if toNext==0:
            toNext=min(toNextX,toNextY)
        #update raypos to border of next grid space
        newPos=[rayPos[0]+toNext*xSlope, rayPos[1]+toNext*ySlope,rayPos[2]]
        eps2=0.001
        #check if still in same grid space due to floats; if is, increment by small amount
        while(newPos[0]/10==rayPos[0]/10 and newPos[1]/10==rayPos[1]/10):
            newPos = [newPos[0]+eps2*xSlope, newPos[1]+eps2*ySlope,rayPos[2]]

        #update rayPos
        rayPos=newPos
        withinX=rayPos[0]>0 and rayPos[0]<maxX
        withinY=rayPos[1]>0 and rayPos[1]<maxY
        notWall = (withinX and withinY and map[int(rayPos[1]/10)][int(rayPos[0]/10)] < 0)

    #return ray length, with 25cm adjustment for laser
    readingLen=math.sqrt((pos[0]-rayPos[0])**2+(pos[1]-rayPos[1])**2)
    return adjuster(readingLen,adj,abs(lAngle))
