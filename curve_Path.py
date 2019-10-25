import math
import vrep
from mpc import mpc_controller
import sys
import matplotlib.pyplot as plt
import datetime
import numpy as np
from scene_constants import scene_constants
from genTraj import genTrajectory
from genTraj import genTrajectoryInit
from collections import deque



Vx=2.77
PRD_HRZ = 10
dt=0.025
SENSOR_COUNT = 19

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

def sensorApp(clientID,queue,colhad, scene):
    _,handles,intD, floatD, stringD = vrep.simxGetObjectGroupData(clientID,colhad,13,vrep.simx_opmode_blocking)
    detection = np.reshape(intD,(SENSOR_COUNT,2))[:,0]
    detection = 1-detection ## 0 to 1 , 1 to 0
    detectedPoint = np.reshape(floatD,(SENSOR_COUNT,6))[:,0:3]

    detDist = np.sum(np.abs(detectedPoint) ** 2, axis=1) ** (1. / 2)
    for index in range(0,len(detection)):
        if detection[index] == 1:
            detDist[index] = scene.max_distance

    nomDetDist = 1/scene.sensor_distance*detDist

    sensorData = np.append(nomDetDist,detection)

    queue.append(sensorData)

    return queue



if __name__ == "__main__":

    vrep.simxFinish(-1)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        print("ERROR: Cannot establish connection to vrep.")
        sys.exit()

    #getPath.setID(clientID)

    # Set Sampling time
    vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, dt, vrep.simx_opmode_oneshot)

    vrep.simxSynchronous(clientID, True)

    ########## Get handles from vrep ###########

    _, vehicle_handle = vrep.simxGetObjectHandle(clientID, "dyros_vehicle0", vrep.simx_opmode_blocking)
    _, goal_handle = vrep.simxGetObjectHandle(clientID, "GoalPoint0",vrep.simx_opmode_blocking)

    _, colHandle = vrep.simxGetCollectionHandle(clientID,"Ref",vrep.simx_opmode_blocking)
    _, sensorColHandle = vrep.simxGetCollectionHandle(clientID,"Sensors",vrep.simx_opmode_blocking)
    _, refHandle, intData, doubleData, strData = vrep.simxGetObjectGroupData(clientID,colHandle,9,vrep.simx_opmode_blocking)

    ref_from_scene_pos = np.reshape(doubleData,(len(refHandle),6))[:,0:3]
    ref_from_scene_ori = np.reshape(doubleData, (len(refHandle),6))[:,3:6]

    _, goalPos = vrep.simxGetObjectPosition(clientID,goal_handle,-1,vrep.simx_opmode_blocking)

    scene_constants.clientID = clientID
    scene_constants.vehicle_handle = vehicle_handle
    scene_constants.goal_handle = goal_handle
    scene_constants.colHandle = colHandle
    scene_constants.refHandle = refHandle
    scene_constants.refPos = ref_from_scene_pos
    scene_constants.refOri = ref_from_scene_ori

    weightFilePath = './2019-09-30_11_17_17.948153_e25_gs1674.h5'
    sample_options, sample_scene_const, network_model = genTrajectoryInit(weightFilePath)
    sample_options.VEH_COUNT =1

    print("getHandles")

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

    sensorQue = deque(maxlen=sample_options.FRAME_COUNT)
    goalQue = deque(maxlen=sample_options.FRAME_COUNT)

    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

    setMotorSpeed(clientID, motor_handle, Vx)
    setMotorPosition(clientID, steer_handle, 0)

    vrep.simxSynchronousTrigger(clientID)

    for i in range(0,sample_options.FRAME_COUNT-1):

        vehLin = np.zeros(3)
        _, vehiclePos = vrep.simxGetObjectPosition(clientID, vehicle_handle, -1, vrep.simx_opmode_blocking)
        _, vehicleOri = vrep.simxGetObjectOrientation(clientID, vehicle_handle, -1, vrep.simx_opmode_blocking)
        _, vehicleLin, vehicleAng = vrep.simxGetObjectVelocity(clientID, vehicle_handle, vrep.simx_opmode_streaming)

        ## global coordinate

        vehLin[0] = (math.cos(vehicleOri[2]) * vehicleLin[0] + math.sin(vehicleOri[2]) * vehicleLin[
            1])  ## Rotate Vx from world frame to vehicle frame
        vehLin[1] = (-math.sin(vehicleOri[2]) * vehicleLin[0] + math.cos(vehicleOri[2]) * vehicleLin[
            1])  ## Rotate Vy form wold frame to vehicle frame
        vehLin[2] = vehicleLin[2]

        goalDist = np.linalg.norm(np.subtract(vehiclePos[0:2], goalPos[0:2]))
        goalHeading = math.atan2(vehiclePos[0]- goalPos[0], vehiclePos[1] - goalPos[1])

        sensorApp(clientID, sensorQue,sensorColHandle,sample_scene_const)
        goalQue.append([goalDist, goalHeading])

        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)

    #vrep.simxSynchronousTrigger(clientID)

    e = np.zeros((4,1))

    print("goalQue :" +str(np.array(sensorQue).T))
    print("sensorQue :" + str(np.array(goalQue).T))

    while not Reach:
        print("-------------------------------------------")

        vehLin = np.zeros(3)
        _, vehiclePos = vrep.simxGetObjectPosition(clientID, vehicle_handle, -1, vrep.simx_opmode_blocking)
        _, vehicleOri = vrep.simxGetObjectOrientation(clientID, vehicle_handle, -1, vrep.simx_opmode_blocking)
        _, vehicleLin, vehicleAng = vrep.simxGetObjectVelocity(clientID, vehicle_handle, vrep.simx_opmode_streaming)

        ## global coordinate

        vehLin[0] = (math.cos(vehicleOri[2]) * vehicleLin[0] + math.sin(vehicleOri[2]) * vehicleLin[1]) ## Rotate Vx from world frame to vehicle frame
        vehLin[1] = (-math.sin(vehicleOri[2]) * vehicleLin[0] + math.cos(vehicleOri[2]) * vehicleLin[1]) ## Rotate Vy form wold frame to vehicle frame
        vehLin[2] = vehicleLin[2]

        goalDist = np.linalg.norm(np.subtract(vehiclePos[0:2], goalPos[0:2]))
        normGoalDist = 1/sample_scene_const.goal_distance*goalDist
        goalHeading = math.atan2(vehiclePos[0]-goalPos[0],vehiclePos[1]-goalPos[1])

        sensorApp(clientID, sensorQue,sensorColHandle, sample_scene_const)
        goalQue.append([normGoalDist,goalHeading])

        if goalDist < 0.5:
            Reach = True
            break

        vehRes = np.vstack((vehRes,vehiclePos))

        refOri = np.zeros(PRD_HRZ)

        sample_veh_pos = np.array([vehiclePos[0:2]])
        sample_veh_heading = np.array([vehicleOri[2]])
        sample_state_sensor = np.array([np.array(sensorQue).T])
        sample_state_goal = np.array([np.array(goalQue).T])

        print("sample_veh_pos : " + str(sample_veh_pos))
        print("sample_veh_heading : " + str(sample_veh_heading))
        print("sample_state_sensor : " + str(sample_state_sensor))
        print("sample_state_goal : " + str(sample_state_goal))


        #refPos, refOri = getPath.pathFromArray(scene_constants)
        traj_est = genTrajectory(sample_options, sample_scene_const, sample_veh_pos, sample_veh_heading,
                                 sample_state_sensor, sample_state_goal, network_model, PRD_HRZ)

        refPos = traj_est[0].T
        #refPos = np.append(traj_est, np.zeros((traj_est[:, 1].size, 1)), axis=1) #### from [[1,1],[2,2]] to [[1,1,0],[2,2,0]]

        print(refPos)

        for index in range(0, len(refPos)-1):
            curPos = refPos[index]
            nextPos = refPos[index+1]

            relPos = nextPos - curPos

            ang = math.atan2(relPos[1], relPos[0])
            refOri[index] = ang

        ori = refOri[0]

        psi = 0
        y = (-math.sin(-ori) * -refPos[0, 0] + math.cos(-ori) * -refPos[0, 1])
        #ydot = (-math.sin(-ori) * vehLin[0] + math.cos(-ori) * vehLin[1])
        ydot = vehLin[1]
        psidot = vehicleAng[2]

        y_des = 0
        psi_des = refOri[0]
        psidot_des = (refOri[1]-refOri[0])/(dt*8)
        input2 = psidot_des*np.ones(PRD_HRZ)

        e[0] = y - y_des
        e[1] = ydot + Vx*(psi - psi_des)
        e[2] = psi - psi_des
        e[3] = psidot - psidot_des

        print("y : " + str(y) + ", y_des : " + str(y_des))
        print("ydot : " + str(ydot))
        print("longitudinal velocity : " + str((math.cos(psi)*vehicleLin[0]+math.sin(psi)*vehicleLin[1])))
        print("e[1] : " + str(e[1]))
        print("psi : " + str(psi) + ", psi_des : "  + str(psi_des))
        print("psidot : " + str(psidot) + ", psidot_des : " + str(psidot_des))
        print("goaldist : " + str(goalDist))

        steer = mpc_controller(e,input2)

        setMotorPosition(clientID,steer_handle,steer)

        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)

    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
    vrep.simxFinish(clientID)

    plt.axis('equal')

    plt.plot(scene_constants.refPos[:,0],scene_constants.refPos[:,1],color='black',linestyle='--',label='Reference')
    plt.scatter(vehRes[1:,0],vehRes[1:,1],label='Vehicle',c='Red',s=0.7)

    plt.xlabel('X position')
    plt.ylabel('Y poistion')

    plt.legend()

    plt.savefig('./Image/'+str(datetime.datetime.now())+'.png', dpi=1200)