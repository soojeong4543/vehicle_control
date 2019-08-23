import vrep
import math

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
    _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [0, 0 + i*0.5, 0], vrep.simx_opmode_oneshot)'''

for i in range(0,100):
    print("i = " + str(i))
    _, dhandle = vrep.simxGetObjectHandle(clientID,"Dummy"+str(100+i),vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectPosition(clientID,dhandle,-1,[0,i*0.5,0],vrep.simx_opmode_streaming)
    _ = vrep.simxSetObjectOrientation(clientID,dhandle,-1,[0,0,math.pi/2],vrep.simx_opmode_oneshot)