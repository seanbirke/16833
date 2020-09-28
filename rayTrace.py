import math
#code heavily based on work from lodev.org/cgtutor/raycasting.html
#note: assumes map pixels are 10 cm
#estimates distance to nearest surface by following traj of x until hitting surface in map
#pos=[x,y,theta] of the robot,lAngle=angle offset of laser
#also adjusts for the ~25cm offset from the actual robot position
def adjuster(reading,adj,angle):
    return math.sqrt(reading**2 + adj**2-2*reading*adj*math.cos(angle))
def rayTrace(pos,lAngle,oMap,certainty):
    #find current block in map
    adj=25
    laserTheta=pos[2]+lAngle
    rayDirX=math.cos(laserTheta)
    rayDirY=math.sin(laserTheta)
    deltaDist=[0,0]
    eps=0.0001
    if rayDirX>=eps and rayDirY>=eps:
        deltaDist=[abs(1/rayDirX),abs(1/rayDirY)]
    res=10
    #initialize rayPos, position of ray in xy
    rayPos=[pos[0],pos[1],laserTheta]
    #position in map coordinates
    mapPos=[int(rayPos[0]/10),int(rayPos[1]/10),laserTheta]
    #length of ray to next x or y side
    sideDistX=0
    sideDistY=0
    #ray direction
    stepX=1
    stepY=1
    side=0
    if rayDirX<0:
        stepX=-1
        sideDistX=(rayPos[0]-mapPos[0]*10)*deltaDist[0];
    else:
        stepX=1
        sideDistX=(mapPos[0]*10-rayPos[0]+10)*deltaDist[0];
    if rayDirY<0:
        stepY=-1
        sideDistY=(rayPos[1]-mapPos[1]*10)*deltaDist[1];
    else:
        stepX=1
        sideDistX=(mapPos[1]*10-rayPos[1]+10)*deltaDist[1];
    maxX=oMap.shape[0]*10
    maxY=oMap.shape[1]*10
    withinX = mapPos[0] >= 0 and mapPos[0] < oMap.shape[0]
    withinY = mapPos[1]>=0 and mapPos[1] < oMap.shape[1]
    notWall = (withinX and withinY and oMap[mapPos[1]][mapPos[0]] < certainty)

    while(withinX and withinY and notWall):
        #jump to next map square or in x-dir or in y-dir
        if sideDistX<sideDistY:
            sideDistX+=deltaDist[0]
            mapPos[0]+=stepX
            side=0
        else:
            sideDistY+=deltaDist[1]
            mapPos[1]+=stepY
            side=1
        withinX = mapPos[0] >= 0 and mapPos[0] < oMap.shape[0]
        withinY = mapPos[1]>=0 and mapPos[1] < oMap.shape[1]
        notWall = (withinX and withinY and oMap[mapPos[1]][mapPos[0]] < certainty)

    readingLen=sideDistX
    if side==1:
        readingLen=sideDistY
    #return ray length, with 25cm adjustment for laser
    finalLen=adjuster(readingLen,adj,abs(lAngle))
    #print(finalLen,withinX,withinY,notWall)
    return finalLen
