import math

#code heavily based on work from lodev.org/cgtutor/raycasting.html
#note: assumes map pixels are 10 cm
#estimates distance to nearest surface by following traj of x until hitting surface in map
#pos=[x,y,theta] of the robot,lAngle=angle offset of laser
#also adjusts for the ~25cm offset from the actual robot position
def adjuster(reading,adj,angle):
	return math.sqrt(reading**2 + adj**2-2*reading*adj*math.cos(math.pi-angle))
def rayTrace(pos,lAngle,oMap,certainty):
	#print("position:",pos,",lAngle=",lAngle)
	#find current block in map
	adj=25
	laserTheta=pos[2]+lAngle
	rayDirX=math.cos(laserTheta)
	rayDirY=math.sin(laserTheta)
	infty=float("inf")
	deltaDist=[infty,infty]
	eps=0.0001
	res=10
	if abs(rayDirX)>eps:
		deltaDist[0]=abs(1/rayDirX)*res
	if abs(rayDirY)>eps:
		deltaDist[1]=abs(1/rayDirY)*res
	res=10
	#initialize rayPos, position of ray in xy
	rayPos=[pos[0],pos[1],laserTheta]
	#position in map coordinates
	mapPos=[int(rayPos[0]/res),int(rayPos[1]/res),laserTheta]
	#length of ray to next x or y side
	sideDistX=0
	sideDistY=0
	#ray direction
	stepX=1
	stepY=1
	side=0
	if rayDirX<0:
		stepX=-1
		sideDistX=(rayPos[0]-mapPos[0]*res)*deltaDist[0]/res;
	else:
		stepX=1
		sideDistX=(mapPos[0]*res-rayPos[0]+res)*deltaDist[0]/res;
	if rayDirY<0:
		stepY=-1
		sideDistY=(rayPos[1]-mapPos[1]*res)*deltaDist[1]/res;
	else:
		stepX=1
		sideDistY=(mapPos[1]*res-rayPos[1]+res)*deltaDist[1]/res;
	pside=side
	#print("sideDistX=",sideDistX,"sideDistY=",sideDistY,"deltaDist=",deltaDist)
	maxX=oMap.shape[0]
	maxY=oMap.shape[1]
	#maxX=len(oMap[0])
	#maxY=len(oMap)
	withinX = mapPos[0] >= 0 and mapPos[0] < maxX
	withinY = mapPos[1]>=0 and mapPos[1] < maxY
	notWall = (withinX and withinY and oMap[mapPos[1]][mapPos[0]] < certainty)
	while(withinX and withinY and notWall):
		#print("loop; mapPos=",mapPos,", (sdX,sdY)=",sideDistX,sideDistY)
		#jump to next map square or in x-dir or in y-dir
		if sideDistX<sideDistY:
			sideDistX+=deltaDist[0]
			mapPos[0]+=stepX
			pSide=side
			side=0
		else:
			sideDistY+=deltaDist[1]
			mapPos[1]+=stepY
			pside=side
			side=1
		withinX = mapPos[0] >= 0 and mapPos[0] < maxX
		withinY = mapPos[1]>=0 and mapPos[1] < maxY
		notWall = (withinX and withinY and oMap[mapPos[1]][mapPos[0]] < certainty)
	#can find length of ray by determining the last valid sideDistX/Y
	readingLen=sideDistX
	if side==0:
		readingLen=sideDistY
		if pside==0:
			if deltaDist[0]==infty:
				print(laserTheta)
				readingLen=sideDistX
			else:
				readingLen=sideDistX-deltaDist[0]
	elif pside==1:
		if deltaDist[1]==infty:
			print(laserTheta,deltaDist)
			readingLen=sideDistY
		else:
			readingLen=sideDistY-deltaDist[1]

	#print(withinX,withinY,notWall,mapPos,readingLen)
	#return ray length, with 25cm adjustment for laser
	finalLen=adjuster(readingLen,adj,abs(lAngle))
	#print(finalLen,withinX,withinY,notWall)
	return finalLen

#m=[[1,1,1,1,1,1,1],[1,0,0,0,0,0,1],[1,0,0,0,0,0,1],[1,0,0,0,0,0,1],[1,0,0,0,0,0,1]]
#p=[11,15,0.7854]
#p=[11,15,0]
#lAng=0
#certain=0.2
#rRes=rayTrace(p,lAng,m,certain)
#print(rRes)
