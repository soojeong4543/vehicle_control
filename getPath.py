import numpy as np
import math
import vrep


#clientID = None
PRD_HRZ = 20

#ref_from_scene_pos = None
#ref_from_scene_ori = None

'''
def setID(ID):
    global clientID
    clientID = ID

    getRef()

def getRef():
    _, colHandle = vrep.simxGetCollectionHandle(clientID,"Ref",vrep.simx_opmode_blocking)
    _, refHandle, intData, doubleData, strData = vrep.simxGetObjectGroupData(clientID,colHandle,9,vrep.simx_opmode_blocking)

    global ref_from_scene_pos
    global ref_from_scene_ori

    ref_from_scene_pos = np.reshape(doubleData,(len(refHandle),6))[:,0:3]
    ref_from_scene_ori = np.reshape(doubleData, (len(refHandle),6))[:,3:6]
'''

def dummy(vehState):
    refPos = np.zeros((0, 3))
    refOri = np.zeros(0)

    for i in range(0,PRD_HRZ):
        refPos = np.vstack((refPos,[(i+1)*0.5,0,0]))

    for index in range(0, len(refPos)-1):
        curPos = refPos[index]
        nextPos = refPos[index+1]

        relPos = nextPos - curPos

        ang = math.atan2(relPos[1], relPos[0])
        refOri = np.append(refOri,ang)

    #refOri[len(refPos)-1] = refOri[len(refPos)-2]
    return refPos, refOri


def pathFromRef(scene): ## Receive position of vehicle [x,y,z] as position vehState\
    refOri = np.zeros(PRD_HRZ)
    refPos = np.zeros((PRD_HRZ, 3))

    _, vehPos = vrep.simxGetObjectPosition(scene.clientID,scene.vehicle_handle,-1,vrep.simx_opmode_blocking)
    _, vehOri = vrep.simxGetObjectOrientation(scene.clientID, scene.vehicle_handle, -1, vrep.simx_opmode_blocking)

    distMtx = scene.refPos - vehPos
    normMtx = np.sum(np.abs(distMtx) ** 2, axis=1) ** (1. / 2)
    nearestRef = np.argmin(normMtx)

    print("nearestRef: " + str(nearestRef))

    if nearestRef+PRD_HRZ+1 > len(distMtx):
        for i in range(0, PRD_HRZ):
            refPos[i] = [(i + 1) * 0.5, 0, 0]
            refOri[i] = 0
            #refPos = np.vstack((refPos, [(i + 1) * 0.5, 0, 0]))
            #refOri = np.append(refOri,0)

    else :
        gloRefPos = scene.refPos[nearestRef:nearestRef+PRD_HRZ]
        relRefPos = gloRefPos - vehPos
        refPos[:, 0] = math.cos(vehOri[2]) * relRefPos[:, 0] + math.sin(vehOri[2]) * relRefPos[:, 1]
        refPos[:, 1] = -math.sin(vehOri[2]) * relRefPos[:, 0] + math.cos(vehOri[2]) * relRefPos[:, 1]

        for index in range(0, len(refPos)-1):
            curPos = refPos[index]
            nextPos = refPos[index+1]

            relPos = nextPos - curPos

            ang = math.atan2(relPos[1], relPos[0])
            refOri[index] = ang
    print("vehPri : " + str(vehOri[2]))
    print("RefPos : "+str(refPos))
    print("RefOri : " + str(refOri))
    return refPos, refOri

'''else :
    gloRefPos = scene.refPos[nearestRef:nearestRef+PRD_HRZ]
    relRefPos = gloRefPos - vehPos
    refPos[:,0] = math.cos(vehOri[2])*relRefPos[:,0] + math.sin(vehOri[2])*relRefPos[:,1]
    refPos[:,1] = -math.sin(vehOri[2]) * relRefPos[:, 0] + math.cos(vehOri[2]) * relRefPos[:, 1]


    for index in range(0, len(refPos) - 1):
        #curPos = scene.refPos[nearestRef + index]
        #nextPos = scene.refPos[nearestRef + index + 1]

        curPos = refPos[index]
        nextPos = refPos[index+1]

        relPos = nextPos - curPos

        ang = math.atan2(relPos[1], relPos[0])
        refOri = np.append(refOri, ang)'''