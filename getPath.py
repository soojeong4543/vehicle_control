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
    #print("vehOri : " + str(vehOri[2]))
    #print("RefPos : "+str(refPos))
    #print("RefOri : " + str(refOri))
    return refPos, refOri

def pathFromArray(scene):
    ref = np.array([[70.48360443115234, -10.043363571166992], [70.96231842041016, -10.161726951599121], [71.43309783935547, -10.276163101196289], [71.93016052246094, -10.30653190612793], [72.45673370361328, -10.292058944702148], [72.98052215576172, -10.227174758911133], [73.49597930908203, -10.1893310546875], [73.97962951660156, -10.276214599609375], [74.45004272460938, -10.406652450561523], [74.94414520263672, -10.524876594543457], [75.43424987792969, -10.685851097106934], [75.87019348144531, -10.91171646118164], [76.26227569580078, -11.169175148010254], [76.65263366699219, -11.40818977355957], [77.08982849121094, -11.55789852142334], [77.55615234375, -11.66977310180664], [78.05715942382812, -11.758158683776855], [78.55437469482422, -11.763738632202148], [79.04425048828125, -11.75006103515625], [79.55038452148438, -11.750432968139648], [80.09205627441406, -11.748528480529785], [80.61286926269531, -11.70164680480957], [81.11235046386719, -11.67890739440918], [81.6207275390625, -11.751220703125], [82.12155151367188, -11.804888725280762], [82.64944458007812, -11.78343391418457], [83.1874771118164, -11.74527359008789], [83.71363830566406, -11.742227554321289], [84.22807312011719, -11.754210472106934], [84.74732208251953, -11.76268482208252], [85.28069305419922, -11.82103157043457], [85.81440734863281, -11.842958450317383], [86.32891082763672, -11.773691177368164], [86.82425689697266, -11.731268882751465], [87.35528564453125, -11.768651962280273], [87.8768081665039, -11.76508903503418], [88.38402557373047, -11.672323226928711], [88.88470458984375, -11.54355239868164], [89.37828063964844, -11.385671615600586], [89.89115142822266, -11.243802070617676], [90.39845275878906, -11.20079231262207], [90.88821411132812, -11.24370002746582], [91.38282012939453, -11.281991958618164], [91.88375854492188, -11.346443176269531], [92.39026641845703, -11.436246871948242], [92.93148803710938, -11.439922332763672], [93.46745300292969, -11.466477394104004], [93.96360778808594, -11.59170913696289], [94.41796875, -11.773030281066895], [94.87660217285156, -11.956520080566406], [95.37159729003906, -12.049848556518555], [95.86819458007812, -12.048249244689941], [96.36312103271484, -12.036590576171875], [96.89824676513672, -12.04439640045166], [97.44175720214844, -12.034486770629883], [97.96321105957031, -11.965801239013672], [98.44532775878906, -11.815926551818848], [98.9144515991211, -11.652695655822754], [99.38386535644531, -11.474187850952148], [99.8655776977539, -11.289396286010742], [100.3499755859375, -11.068475723266602], [100.82816314697266, -10.83260440826416], [101.31632995605469, -10.648340225219727], [101.8195571899414, -10.454683303833008], [102.27540588378906, -10.182363510131836], [102.71690368652344, -9.934608459472656], [103.1872787475586, -9.792360305786133], [103.67044830322266, -9.723027229309082], [104.1483383178711, -9.70012092590332], [104.61625671386719, -9.719465255737305], [105.0674819946289, -9.78372573852539], [105.51939392089844, -9.834114074707031], [105.99371337890625, -9.849336624145508], [106.45845794677734, -9.954143524169922], [106.89631652832031, -10.118913650512695], [107.33675384521484, -10.30117416381836], [107.79024505615234, -10.502473831176758], [108.2145004272461, -10.74312973022461], [108.6457748413086, -10.991060256958008], [109.1346206665039, -11.16221809387207], [109.62361907958984, -11.326659202575684], [110.045166015625, -11.589800834655762], [110.41670989990234, -11.906034469604492], [110.75172424316406, -12.244046211242676], [111.048828125, -12.605122566223145], [111.30680847167969, -12.986688613891602], [111.56260681152344, -13.387388229370117], [111.81412506103516, -13.805784225463867], [112.02320861816406, -14.249467849731445], [112.20279693603516, -14.710698127746582], [112.400146484375, -15.186261177062988], [112.6461181640625, -15.648408889770508], [112.96440124511719, -16.058368682861328], [113.25709533691406, -16.460124969482422], [113.4604263305664, -16.90797233581543], [113.58128356933594, -17.378156661987305], [113.65345764160156, -17.845163345336914], [113.68306732177734, -18.30344009399414], [113.72483825683594, -18.77877426147461], [113.86720275878906, -19.25872802734375], [114.01347351074219, -19.742183685302734], [114.0609130859375, -20.241132736206055], [114.02716064453125, -20.72726058959961], [113.94581604003906, -21.19559097290039], [113.82608795166016, -21.642986297607422], [113.71174621582031, -22.111682891845703], [113.68724060058594, -22.62125015258789], [113.7381820678711, -23.134777069091797], [113.78480529785156, -23.65229606628418], [113.79560852050781, -24.188472747802734], [113.8413314819336, -24.721302032470703], [113.89945983886719, -25.224790573120117], [113.848876953125, -25.71298599243164], [113.7216796875, -26.180755615234375], [113.61064147949219, -26.655847549438477], [113.58528137207031, -27.164066314697266], [113.63521575927734, -27.68987464904785], [113.64878845214844, -28.197717666625977], [113.55086517333984, -28.68340492248535], [113.38514709472656, -29.13999366760254], [113.18994140625, -29.568618774414062], [112.95401763916016, -29.97178077697754], [112.68675231933594, -30.351760864257812], [112.3946762084961, -30.702611923217773], [112.07276916503906, -31.024438858032227], [111.71907043457031, -31.322402954101562], [111.3432388305664, -31.5884952545166], [110.95207214355469, -31.81477928161621], [110.5424575805664, -32.00353240966797], [110.1115493774414, -32.161285400390625], [109.6617202758789, -32.317752838134766], [109.1977767944336, -32.46274185180664], [108.716796875, -32.57912826538086], [108.21878814697266, -32.66992950439453], [107.72151947021484, -32.79103088378906], [107.23779296875, -33.00996398925781], [106.76924896240234, -33.214847564697266], [106.27542877197266, -33.31484603881836], [105.7730484008789, -33.354164123535156], [105.2547607421875, -33.42268753051758], [104.75367736816406, -33.585330963134766], [104.25460052490234, -33.741031646728516], [103.724365234375, -33.823326110839844], [103.20402526855469, -33.95111083984375], [102.69474029541016, -34.09698486328125], [102.18763732910156, -34.13155746459961], [101.68489074707031, -34.107643127441406], [101.1775131225586, -34.117713928222656], [100.65972137451172, -34.21376037597656], [100.14649963378906, -34.35010528564453], [99.65451049804688, -34.53399658203125], [99.17633056640625, -34.68347930908203], [98.68731689453125, -34.729984283447266], [98.20260620117188, -34.699195861816406], [97.71418762207031, -34.676273345947266], [97.22126007080078, -34.753990173339844], [96.73469543457031, -34.90013885498047], [96.2594985961914, -35.008087158203125], [95.78112030029297, -35.065853118896484], [95.296875, -35.15769958496094], [94.80851745605469, -35.14619827270508], [94.33484649658203, -35.066505432128906], [93.87272644042969, -34.99971389770508], [93.38893127441406, -35.02766418457031], [92.89219665527344, -35.128746032714844], [92.39594268798828, -35.25816345214844], [91.90369415283203, -35.418601989746094], [91.41744232177734, -35.54914093017578], [90.9247817993164, -35.572105407714844], [90.42350769042969, -35.58278274536133], [89.888916015625, -35.65504837036133], [89.37332916259766, -35.67894744873047], [88.87792205810547, -35.606536865234375], [88.40034484863281, -35.54820251464844], [87.90975952148438, -35.592185974121094], [87.40974426269531, -35.700782775878906], [86.9222640991211, -35.778663635253906], [86.4358901977539, -35.75175857543945], [85.96199035644531, -35.645172119140625], [85.47952270507812, -35.544864654541016], [84.97261047363281, -35.53610610961914], [84.46476745605469, -35.59981918334961], [83.96646881103516, -35.694244384765625], [83.45624542236328, -35.76622772216797], [82.9061508178711, -35.80878448486328], [82.36503601074219, -35.816349029541016], [81.86155700683594, -35.72869110107422], [81.3629150390625, -35.64048767089844], [80.84954071044922, -35.650413513183594], [80.34046936035156, -35.72712326049805], [79.83841705322266, -35.83423614501953], [79.33289337158203, -35.97285842895508], [78.82614135742188, -36.082313537597656], [78.32759094238281, -36.07878494262695], [77.82254791259766, -36.05376434326172], [77.31333923339844, -36.1246452331543], [76.81510162353516, -36.200565338134766], [76.32213592529297, -36.16965866088867], [75.8325424194336, -36.122886657714844], [75.33052062988281, -36.17500686645508], [74.81961059570312, -36.23408126831055], [74.29530334472656, -36.314109802246094], [73.77143859863281, -36.41035842895508], [73.23946380615234, -36.43766784667969], [72.73597717285156, -36.3715705871582], [72.23157501220703, -36.30341720581055], [71.68855285644531, -36.29311752319336], [71.1539077758789, -36.24127960205078], [70.6655502319336, -36.09737777709961], [70.17716979980469, -35.954811096191406], [69.64387512207031, -35.86513900756836], [69.1324691772461, -35.73194885253906], [68.6794662475586, -35.515663146972656], [68.26911926269531, -35.25716018676758], [67.890869140625, -34.9686279296875], [67.54594421386719, -34.649375915527344], [67.18457794189453, -34.33719253540039], [66.74860382080078, -34.0401496887207], [66.28694152832031, -33.730613708496094], [65.8261489868164, -33.42162322998047], [65.36503601074219, -33.11058044433594], [64.90348815917969, -32.79789733886719], [64.44258880615234, -32.4892692565918], [63.98284149169922, -32.18072509765625], [63.523895263671875, -31.869491577148438], [63.06267166137695, -31.559438705444336], [62.60163116455078, -31.25164222717285], [62.12984085083008, -30.987300872802734], [61.63560104370117, -30.795385360717773], [61.141754150390625, -30.58246421813965], [60.65093994140625, -30.331039428710938], [60.143577575683594, -30.12742042541504], [59.6220703125, -29.995529174804688], [59.093345642089844, -29.896434783935547], [58.562835693359375, -29.829208374023438], [58.02935791015625, -29.796585083007812], [57.495635986328125, -29.796222686767578], [56.96965408325195, -29.7739315032959], [56.42790603637695, -29.768383026123047], [55.8970832824707, -29.8341064453125], [55.372703552246094, -29.929309844970703], [54.845375061035156, -30.008533477783203], [54.29163360595703, -30.05056381225586], [53.742408752441406, -30.09977149963379], [53.2114143371582, -30.107280731201172], [52.71459197998047, -30.027246475219727], [52.25090789794922, -29.892353057861328], [51.811065673828125, -29.71685791015625], [51.39653778076172, -29.503997802734375], [50.966880798339844, -29.297733306884766], [50.47269821166992, -29.133243560791016], [49.96839141845703, -29.01241683959961], [49.471031188964844, -28.995128631591797], [48.98480987548828, -29.035348892211914], [48.49458694458008, -29.065715789794922], [47.977630615234375, -29.07843589782715], [47.43659973144531, -29.114044189453125]])
    for i in range(0, PRD_HRZ):
        ref = np.append(ref,[[ref[-1][0]+scene.Vx*scene.CTR_FRQ,ref[-1][1]]],axis=0)

    ref = np.append(ref, np.zeros((ref[:, 1].size, 1)), axis=1) #### from [[1,1],[2,2]] to [[1,1,0],[2,2,0]]

    refOri = np.zeros(PRD_HRZ)
    refPos = np.zeros((PRD_HRZ, 3))

    _, vehPos = vrep.simxGetObjectPosition(scene.clientID,scene.vehicle_handle,-1,vrep.simx_opmode_blocking)
    _, vehOri = vrep.simxGetObjectOrientation(scene.clientID, scene.vehicle_handle, -1, vrep.simx_opmode_blocking)

    distMtx = ref - vehPos
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
        gloRefPos = ref[nearestRef:nearestRef+PRD_HRZ]
        relRefPos = gloRefPos - vehPos
        refPos[:, 0] = math.cos(vehOri[2]) * relRefPos[:, 0] + math.sin(vehOri[2]) * relRefPos[:, 1]
        refPos[:, 1] = -math.sin(vehOri[2]) * relRefPos[:, 0] + math.cos(vehOri[2]) * relRefPos[:, 1]

        for index in range(0, len(refPos)-1):
            curPos = refPos[index]
            nextPos = refPos[index+1]

            relPos = nextPos - curPos

            ang = math.atan2(relPos[1], relPos[0])
            refOri[index] = ang
    #print("vehOri : " + str(vehOri[2]))
    #print("RefPos : "+str(refPos))
    #print("RefOri : " + str(refOri))
    return refPos, refOri