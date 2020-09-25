import math
#note: assumes map pixels are 10 cm
#estimates distance to nearest surface by following traj of x until hitting surface in map
#pos=[x,y,theta] of the LASER, not the robot
#also adjusts for the ~25cm offset from the actual robot position
def rayTrace(pos,map):
    #find current block in map
    adj=25
    xSlope=math.cos(pos[2])
    ySlope=math.sin(pos[2])
    #put 25cm adjustment for laser
    rayPos=[pos[0]-adj*xSlope,pos[1]-adj*ySlope,pos[2]]
    res=10
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
        #return ray length
    return math.sqrt((pos[0]-rayPos[0])**2+(pos[1]-rayPos[1])**2)
