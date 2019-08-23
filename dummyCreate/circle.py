import vrep
import math
import numpy as np

R=15

clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
_, dsethandle = vrep.simxCreateDummy(clientID, 0.5, None, vrep.simx_opmode_blocking)

for x in np.arange(0,R*2,0.5):
    _, dhandle = vrep.simxCreateDummy(clientID, 0.3, [255,0,0], vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectParent(clientID,dhandle,dsethandle,True,vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [x+25 ,math.sqrt(R**2-(x-R)**2), 0], vrep.simx_opmode_oneshot)

for x in np.arange(2*R,0,-0.5):
    _, dhandle = vrep.simxCreateDummy(clientID, 0.3, [255,0,0], vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectParent(clientID,dhandle,dsethandle,True,vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [x+25 ,-math.sqrt(R**2-(x-R)**2), 0], vrep.simx_opmode_oneshot)



_, colHandle = vrep.simxGetCollectionHandle(clientID,"Ref",vrep.simx_opmode_blocking)
_, refHandle, intData, doubleData, strData = vrep.simxGetObjectGroupData(clientID,colHandle,3,vrep.simx_opmode_blocking)

refPos = np.reshape(doubleData,(len(refHandle),3))

for index in range(0,len(refHandle)-1):
    curPos = refPos[index]
    nextPos = refPos[index+1]

    relPos = nextPos-curPos

    ang = math.atan2(relPos[1],relPos[0])

    _ = vrep.simxSetObjectOrientation(clientID,refHandle[index],-1,[0,0,ang],vrep.simx_opmode_blocking)