import vrep
import math
import numpy as np

R = 30

clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

_, dsethandle = vrep.simxCreateDummy(clientID, 0.5, None, vrep.simx_opmode_blocking)

'''
for i in range(0,100):
    _, dhandle = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectParent(clientID,dhandle,dsethandle,True,vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [-50 + i * 0.5, 0, 0], vrep.simx_opmode_oneshot)

for i in range(0,100):
    _, dhandle = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectParent(clientID, dhandle, dsethandle, True, vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [0, 0 + i*0.5, 0], vrep.simx_opmode_oneshot)

for i in range(0,100):
    print("i = " + str(i))
    _, dhandle = vrep.simxGetObjectHandle(clientID,"Dummy"+str(100+i),vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectPosition(clientID,dhandle,-1,[0,i*0.5,0],vrep.simx_opmode_streaming)
    _ = vrep.simxSetObjectOrientation(clientID,dhandle,-1,[0,0,math.pi/2],vrep.simx_opmode_oneshot)
'''
'''
for theta in np.arange(0,2*math.pi,0.5/R):
    _, dhandle = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectParent(clientID, dhandle, dsethandle, True, vrep.simx_opmode_blocking)
    x = R*math.cos(theta)
    y = R*math.sin(theta)
    _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [x, y, 0], vrep.simx_opmode_oneshot)
'''

_, colHandle = vrep.simxGetCollectionHandle(clientID,"Ref",vrep.simx_opmode_blocking)
_, refHandle, intData, doubleData, strData = vrep.simxGetObjectGroupData(clientID,colHandle,3,vrep.simx_opmode_blocking)

refPos = np.reshape(doubleData,(len(refHandle),3))

for index in range(0,len(refHandle)-1):
    curPos = refPos[index]
    nextPos = refPos[index+1]

    relPos = nextPos-curPos

    ang = math.atan2(relPos[1],relPos[0])

    _ = vrep.simxSetObjectPosition(clientID,refHandle[index],-1,[curPos[0],curPos[1],0.1],vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectOrientation(clientID,refHandle[index],-1,[0,0,ang],vrep.simx_opmode_blocking)