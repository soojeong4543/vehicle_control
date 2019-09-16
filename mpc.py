import math
from cvxpy import *
from control import *
import vrep

mass = 740 ## kg
Ca=50000 ## N
lf=lr=1.42 ## m
Iz=3.8*mass ## kg*m^2

max_steer = math.pi/6
min_steer = -math.pi/6

dt = 0.025*4
Vx = 5 ## m/s
R = 20 ## radius of road

PRD_HRZ=20

Ac = np.array([[0, 1, 0, 0,],
               [0,-1*(2*Ca+2*Ca)/mass/Vx,(2*Ca+2*Ca)/mass,(-2*Ca*lf+2*Ca*lr)/mass/Vx],
               [0,0,0,1],
               [0,-1*(2*Ca*lf-2*Ca*lr)/Iz/Vx,(2*Ca*lf-2*Ca*lr)/Iz,-1*(2*Ca*lf*lf+2*Ca*lr*lr)/Iz/Vx]])
Bc = np.array([[0, 0], [2*Ca/mass,-1*(2*Ca*lf-2*Ca*lr)-Vx],[0, 0],[2*Ca*lf/Iz, -1*(2*Ca*lf*lf+2*Ca*lr*lr)/Iz/Vx]])
Cc = np.eye(4)
Dc = np.zeros((4,2))

Q=np.diag([0.5,0.5,0.5,0.5])

sysc = ss(Ac,Bc,Cc,Dc)

sysd = c2d(sysc,dt,method='zoh')

[A,B,C,D] = ssdata(sysd)

e = Variable((4,PRD_HRZ+1))
steer = Variable((1,PRD_HRZ))

e_init = Parameter((4,))
#psidot_des = Parameter(1)
psidot_des = Parameter((1,PRD_HRZ))

objective = 0
constraints = [e[:,0] == e_init]

for i in range(0,PRD_HRZ):
    constraints += [e[:,i+1] == A*e[:,i] + B[:,0]*steer[:,i] + B[:,1]*psidot_des[:,i]]
    constraints += [min_steer <= steer[:,i], steer[:,i] <= max_steer]

    objective += quad_form(e[:,i+1],Q)

prob = Problem(Minimize(objective),constraints)

def mpc_controller(states,input2):
    e_init.value=states[:,0]
    input2 = np.array(input2)[np.newaxis]
    #input2 = input2.T
    psidot_des.value=input2

    prob.solve(solver=OSQP, verbose=False)

    print("optimal steering : " + str(steer.value[:,0]))

    return steer.value[:,0]
