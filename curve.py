#from control import *
import math
import vrep
from mpc import mpc_controller
import sys
import matplotlib.pyplot as plt
import datetime
import scene_constants
import numpy as np
from getPath import dummy
Vx=5
PRD_HRZ = 20
dt=0.025

Reach = False
vehRes = np.zeros(shape=(0,3))

#########################
# Vehicle Control
#########################

# Set speed of motors
# Input:
#   clientID    : client ID of vrep instance
#   motorHandles: list of integers, denoting motors that you want to change the speed
#   desiredSpd  : single number, speed in m/sec
def setMotorSpeed(clientID, motorHandles, desiredSpd):
    wheel_radius = 0.63407 * 0.5  # Wheel radius in metre

    desiredSpd_rps = desiredSpd * (1 / wheel_radius)  # m/s into radians per second

    # print("Desired Speed: " + str(desiredSpd) + " km/hr = " + str(desiredSpd_rps) + " radians per seconds = " + str(math.degrees(desiredSpd_rps)) + "degrees per seconds. = " + str(desiredSpd*(1000/3600)) + "m/s" )
    err_code = []
    for mHandle in motorHandles:
        err_code.append(vrep.simxSetJointTargetVelocity(clientID, mHandle, desiredSpd_rps, vrep.simx_opmode_blocking))
        #vrep.simxSetObjectFloatParameter(clientID, mHandle, vrep.sim_shapefloatparam_init_velocity_g, desiredSpd_rps,
                                         #vrep.simx_opmode_blocking)

    return err_code;


# Set Position of motors
# Input:
#   clientID    : client ID of vrep instance
#   motorHandles: 2D list of integers.
#                 [veh1 left, veh1 right]
#                 [veh2 left, veh2 right]...
#   desiredPos  : list of numbers, position in RADIANS
def setMotorPosition(clientID, motorHandles, desiredPos):
    # Sanity check
    #if motorHandles.size != 2 * desiredPos.size:
        #raise ValueError('input to setMotorPosition is not correct! motorHandles must have 2*size of desiredPos.')

    # print(np.reshape( motorHandles, -1) )
    # print( desiredPos)
    # print( np.radians(desiredPos))
    emptyBuff = bytearray()
    for mHandle in motorHandles:
        _ = vrep.simxSetJointTargetPosition(clientID, mHandle, desiredPos, vrep.simx_opmode_blocking)
    #_, _, _, _, _ = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript,
                                                # 'setJointPos_function', np.reshape(motorHandles, -1),
                                                #np.radians(desiredPos), [], emptyBuff, vrep.simx_opmode_blocking)

    return

if __name__ == "__main__":

    vrep.simxFinish(-1)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        print("ERROR: Cannot establish connection to vrep.")
        sys.exit()

    # Set Sampling time
    vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, dt, vrep.simx_opmode_oneshot)

    vrep.simxSynchronous(clientID, True)

    ########## Get handles from vrep ###########

    _, vehicle_handle = vrep.simxGetObjectHandle(clientID, "dyros_vehicle0", vrep.simx_opmode_blocking)

    motor_handle = np.zeros(2, dtype=int)
    steer_handle = np.zeros(2, dtype=int)

    # Get Motor Handles

    _,h1  = vrep.simxGetObjectHandle(clientID, "nakedCar_motorLeft0", vrep.simx_opmode_blocking)
    _,h2  = vrep.simxGetObjectHandle(clientID, "nakedCar_motorRight0", vrep.simx_opmode_blocking)
    _,h3  = vrep.simxGetObjectHandle(clientID, "nakedCar_freeAxisLeft0", vrep.simx_opmode_blocking)
    _,h4  = vrep.simxGetObjectHandle(clientID, "nakedCar_freeAxisRight0", vrep.simx_opmode_blocking)
    _,h5  = vrep.simxGetObjectHandle(clientID, "nakedCar_steeringLeft0", vrep.simx_opmode_blocking)
    _,h6  = vrep.simxGetObjectHandle(clientID, "nakedCar_steeringRight0" , vrep.simx_opmode_blocking)

    motor_handle[0] = h3
    motor_handle[1] = h4

    steer_handle[0] = h5
    steer_handle[1] = h6

    # Get Reference Handles

    _, colHandle = vrep.simxGetCollectionHandle(clientID,"Ref",vrep.simx_opmode_blocking)
    _, refHandle, intData, doubleData, strData = vrep.simxGetObjectGroupData(clientID,colHandle,9,vrep.simx_opmode_blocking)

    refPos = np.reshape(doubleData,(len(refHandle),6))[:,0:3]
    refOri = np.reshape(doubleData, (len(refHandle),6))[:,3:6]

    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

    setMotorSpeed(clientID, motor_handle, Vx)
    setMotorPosition(clientID, steer_handle, 0)

    vrep.simxSynchronousTrigger(clientID)

    e = np.zeros((4,1))

    while not Reach:
        _, vehiclePos = vrep.simxGetObjectPosition(clientID, vehicle_handle, -1, vrep.simx_opmode_streaming)
        _, vehicleOri = vrep.simxGetObjectOrientation(clientID, vehicle_handle, -1, vrep.simx_opmode_streaming)

        vehRes = np.vstack((vehRes,vehiclePos))

        _, vehicleLin, vehicleAng = vrep.simxGetObjectVelocity(clientID, vehicle_handle, vrep.simx_opmode_streaming)

        distMtx = vehiclePos - refPos
        normMtx = np.sum(np.abs(distMtx)** 2,axis=1)**(1./2) ## 2-norm of each row
        nearestRef = np.argmin(normMtx) ## Index number of the nearest dummy

        if nearestRef == len(refHandle)-2:
            Reach = True

        if len(refHandle)-nearestRef <= PRD_HRZ :
            cRef = nearestRef
            fRef = nearestRef
        else :
            cRef = nearestRef + int(PRD_HRZ/2)
            fRef = nearestRef + PRD_HRZ

        cRefOri = refOri[cRef]
        fRefOri = refOri[fRef]
        nRefOri = refOri[nearestRef]

        ori = nRefOri
        #ori = cRefOri

        #y = (-math.sin(ori[2]) * distMtx[cRef,0] + math.cos(ori[2]) * distMtx[cRef,1])
        y = (-math.sin(ori[2]) * distMtx[nearestRef, 0] + math.cos(ori[2]) * distMtx[nearestRef, 1])
        psi = vehicleOri[2] # in radians
        ydot = (-math.sin(ori[2]) * vehicleLin[0] + math.cos(ori[2]) * vehicleLin[1])
        psidot = vehicleAng[2]

        y_des = 0
        psi_des = ori[2]
        #psidot_des = (fRefOri[2]-nRefOri[2])/(dt*4*PRD_HRZ)
        psidot_des = (refOri[nearestRef+1,2]-nRefOri[2])/(dt*4)
        input2 = psidot_des*np.ones(PRD_HRZ)
        #input2 = (refOri[nearestRef+1:nearestRef+21,2]-refOri[nearestRef:nearestRef+20,2])/(dt*4)


        #psidot_des = (refOri[cRef + 1, 2] - cRefOri[2]) / (dt * 4)
        #psidot_des = (refOri[nearestRef+1:fRef+1,2] - refOri[nearestRef:fRef,2])/(dt*4)
        e[0] = y - y_des
        e[1] = ydot + Vx*(psi - psi_des)
        e[2] = psi - psi_des
        e[3] = psidot - psidot_des
        #e[3] = psidot - refOri[nearestRef+1]



        print("-------------------------------------------")
        print("NearestRef : " + str(nearestRef)+", CRef : " + str(cRef) +", fRef : " + str(fRef))
        print("y : " + str(y) + ", y_des : " + str(y_des))
        print("ydot : " + str(ydot))
        print("longitudinal velocity : " + str((math.cos(psi)*vehicleLin[0]+math.sin(psi)*vehicleLin[1])))
        print("e[1] : " + str(e[1]))

        print("psi : " + str(psi) + ", psi_des : "  + str(psi_des))
        print("psidot : " + str(psidot) + ", psidot_des : " + str(psidot_des))
        print("fRefOri[2] : " + str(fRefOri[2]) + ", nRefOri[2]" + str(nRefOri[2]))

        steer = mpc_controller(e,input2)

        setMotorPosition(clientID,steer_handle,steer)

        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)

    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
    vrep.simxFinish(clientID)

    plt.axis('equal')

    plt.plot(refPos[:,0],refPos[:,1],color='black',linestyle='--',label='Reference')
    plt.scatter(vehRes[1:,0],vehRes[1:,1],label='Vehicle',c='Red',s=0.7)

    plt.xlabel('X position')
    plt.ylabel('Y poistion')

    plt.legend()

    plt.savefig('./Image/'+str(datetime.datetime.now())+'.png', dpi=1200)


