from control import *
import math
import vrep
from mpc import mpc_controller
import sys


Vx=5
R=15

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
        vrep.simxSetObjectFloatParameter(clientID, mHandle, vrep.sim_shapefloatparam_init_velocity_g, desiredSpd_rps,
                                         vrep.simx_opmode_blocking)

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
    vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, 0.025,
                                      vrep.simx_opmode_oneshot)

    # start simulation
    vrep.simxSynchronous(clientID, True)

    ########## Get handles from vrep ###########

    err_code, vehicle_handle = vrep.simxGetObjectHandle(clientID, "dyros_vehicle0", vrep.simx_opmode_blocking)

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

    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

    setMotorSpeed(clientID, motor_handle, Vx)
    setMotorPosition(clientID, steer_handle, 0)

    vrep.simxSynchronousTrigger(clientID)

    e = np.zeros((4,1))

    while True:
        _, vehicle_lin, vehicle_ang = vrep.simxGetObjectVelocity(clientID,vehicle_handle,vrep.simx_opmode_streaming)
        _, vehicle_pos = vrep.simxGetObjectPosition(clientID,vehicle_handle,-1,vrep.simx_opmode_streaming)
        _, vehicle_ori = vrep.simxGetObjectOrientation(clientID,vehicle_handle,-1,vrep.simx_opmode_streaming)

        atan = math.atan2(vehicle_pos[1], vehicle_pos[0])

        y = vehicle_pos[1]
        #y = np.linalg.norm(vehicle_pos)
        psi = vehicle_ori[2] # in radians
        ydot = (-math.sin(atan) * vehicle_lin[0] + math.cos(atan) * vehicle_lin[1])
        psidot = vehicle_ang[2]

        y_des = 0
        psi_des =0
        psidot_des = 0

        #y_des = R
        #psi_des = atan + math.pi/2
        #if psi_des >= math.pi : pse_des -= 2*math.pi
        #psidot_des = Vx/R



        e[0] = y - y_des
        e[1] = ydot + Vx*(psi - psi_des)
        e[2] = psi - psi_des
        e[3] = psidot - psidot_des

        print("-------------------------------------------")
        print("y : " + str(y) + ", y_des : " + str(y_des))
        print("ydot : " + str(ydot))
        #print("longitudinal velocity : " + str((-math.sin(psi)*vehicle_lin[0]+math.cos(psi)*vehicle_lin[1])))
        print("longitudinal velocity : " + str((math.cos(psi)*vehicle_lin[0]+math.sin(psi)*vehicle_lin[1])))
        print("e[1] : " + str(e[1]))

        print("psi : " + str(psi) + ", psi_des : "  + str(psi_des))
        print("psidot : " + str(psidot) + ", psidot_des : " + str(psidot_des))

        steer = mpc_controller(e,psidot_des)

        setMotorPosition(clientID,steer_handle,steer)

        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxSynchronousTrigger(clientID)