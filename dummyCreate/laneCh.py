import vrep
import math
import numpy as np

clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
_, dsethandle = vrep.simxCreateDummy(clientID, 0.5, None, vrep.simx_opmode_blocking)

for x in np.arange(0,120,0.5):
    if x <  20:
        _, dhandle = vrep.simxCreateDummy(clientID, 0.3, [255,0,0], vrep.simx_opmode_blocking)
        _ = vrep.simxSetObjectParent(clientID,dhandle,dsethandle,True,vrep.simx_opmode_blocking)
        _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [x ,0, 0], vrep.simx_opmode_oneshot)
    #elif x < 30 :
    #    _, dhandle = vrep.simxCreateDummy(clientID, 0.3, [255,0,0], vrep.simx_opmode_blocking)
    #    _ = vrep.simxSetObjectParent(clientID, dhandle, dsethandle, True, vrep.simx_opmode_blocking)
    #    _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [x, 0.615*math.exp((x-25)/2), 0], vrep.simx_opmode_oneshot)

    elif x < 40 :
        _, dhandle = vrep.simxCreateDummy(clientID, 0.3, [255, 0, 0], vrep.simx_opmode_blocking)
        _ = vrep.simxSetObjectParent(clientID, dhandle, dsethandle, True, vrep.simx_opmode_blocking)
        _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [x, 3/4*(x-20), 0],
                                       vrep.simx_opmode_oneshot)

    elif x < 70 :
        _, dhandle = vrep.simxCreateDummy(clientID, 0.3, [255,0,0], vrep.simx_opmode_blocking)
        _ = vrep.simxSetObjectParent(clientID, dhandle, dsethandle, True, vrep.simx_opmode_blocking)
        _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [x, 15, 0], vrep.simx_opmode_oneshot)

    #elif x < 80 :
    #    _, dhandle = vrep.simxCreateDummy(clientID, 0.3, [255,0,0], vrep.simx_opmode_blocking)
    #    _ = vrep.simxSetObjectParent(clientID, dhandle, dsethandle, True, vrep.simx_opmode_blocking)
    #    _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [x,-0.615*math.exp((x-75)/2)+15, 0], vrep.simx_opmode_oneshot)

    elif x < 90:
        _, dhandle = vrep.simxCreateDummy(clientID, 0.3, [255, 0, 0], vrep.simx_opmode_blocking)
        _ = vrep.simxSetObjectParent(clientID, dhandle, dsethandle, True, vrep.simx_opmode_blocking)
        _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [x, -3/4*(x-70)+15, 0],
                                       vrep.simx_opmode_oneshot)

    else :
        _, dhandle = vrep.simxCreateDummy(clientID, 0.3, [255,0,0], vrep.simx_opmode_blocking)
        _ = vrep.simxSetObjectParent(clientID, dhandle, dsethandle, True, vrep.simx_opmode_blocking)
        _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [x, 0, 0],
                                       vrep.simx_opmode_oneshot)


_, colHandle = vrep.simxGetCollectionHandle(clientID,"Ref",vrep.simx_opmode_blocking)
_, refHandle, intData, doubleData, strData = vrep.simxGetObjectGroupData(clientID,colHandle,3,vrep.simx_opmode_blocking)

refPos = np.reshape(doubleData,(len(refHandle),3))

for index in range(0,len(refHandle)-1):
    curPos = refPos[index]
    nextPos = refPos[index+1]

    relPos = nextPos-curPos

    ang = math.atan2(relPos[1],relPos[0])

    _ = vrep.simxSetObjectOrientation(clientID,refHandle[index],-1,[0,0,ang],vrep.simx_opmode_blocking)