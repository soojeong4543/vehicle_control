import vrep

clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

for i in range(0,int(50/0.5)):
    _, dhandle = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_blocking)
    _ = vrep.simxSetObjectPosition(clientID, dhandle, -1, [-50 + i * 0.5, 0, 0], vrep.simx_opmode_oneshot)

vrep.simob