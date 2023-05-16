import numpy as np
from scipy.integrate import solve_ivp
import math
from json import dump
from path import *
from random import random

g = np.array([0, 0, -9.81])
droneM = 0.5
payloadM = 5
ropeLength = 1
payloadW = 0.5
simulated_drag = 6
droneOffsets = np.array([[payloadW/2, payloadW/2, 0], [-payloadW/2, payloadW/2, 0], [-payloadW/2, -payloadW/2, 0], [payloadW/2, -payloadW/2, 0]])


class Storage():
    def __init__(self):
        self.trackedForce = np.array([0, 0, 0], dtype=np.float64)

global storage 

storage = Storage()

def solveForAccel(payloadPos, payloadVel, dronePos, droneVel, payloadAccel):
    y = dronePos - payloadPos
    y_dot = droneVel - payloadVel
    b = -y_dot.T.dot(y_dot) + y.dot(payloadAccel)
    
    A = np.array([y])
    
    y_norm = y / np.linalg.norm(y)
    
    desAccel = desiredAccel
    if np.linalg.norm(desAccel) == 0: 
        desAccel = np.array([1e-9, 1e-9, 1e-9])

    ctrl = np.cross(y_norm, np.cross(desAccel, y_norm))  
    ctrl_nrm = (ctrl / np.linalg.norm(ctrl))
    
    avel_mag = np.dot(droneVel - payloadVel, ctrl_nrm)
    avel = np.dot(avel_mag, ctrl_nrm)

    ctrl -= avel*simulated_drag
        
    pseudo_inv = np.matmul(A.T, np.linalg.inv(np.matmul(A, A.T)))
    out = np.dot(pseudo_inv, b)

    return out.T[0] + ctrl

def state_dot(t, state):
    payloadPos = np.array(state[0:3], dtype=np.float64)
    dronePos = np.array(state[3:6], dtype=np.float64)
    payloadVel = np.array(state[6:9], dtype=np.float64)
    droneVel = np.array(state[9:12], dtype=np.float64)
    
    rope = dronePos - payloadPos
    ropeDir = rope / np.linalg.norm(rope)

    tensionZ = (desiredAccel - g)[2] * payloadM
    tensionTotal = ropeDir * (tensionZ / ropeDir[2])

    totalPForces = tensionTotal + g * payloadM
    payloadAccel = totalPForces / payloadM
    
    droneAccel = solveForAccel(payloadPos, payloadVel, dronePos, droneVel, payloadAccel)
    
    tension = tensionTotal / 4

    fDrone = droneAccel * droneM
    storage.trackedForce = fDrone + tension - g * droneM
        
    return [*payloadVel] + [*droneVel] + [*payloadAccel] + [*droneAccel]

def gen():
    SCL = 6
    return ((random() - 0.5) * 2) * SCL


# for i in range(20):
#     simulated_drag = i
#     res = solve_ivp(state_dot, [0, 10], s0, rtol=1e-9, atol=1e-9)
#     print(i, len(res.y[0])) 

payloadPos = np.array([0, 0, payloadW], dtype=np.float64)
payloadVel = np.array([0, 0, 0], dtype=np.float64)
offset = 0
dronePos = np.array(payloadPos + [offset, offset, math.sqrt(ropeLength * ropeLength - 2 * offset * offset)], dtype=np.float64)
droneVel = np.array([0, 0, 0], dtype=np.float64)

s0 = [*payloadPos] + [*dronePos] + [*payloadVel] + [*droneVel]



d = {"payload": [], "drone1": [], "drone2": [], "drone3": [], "drone4": []}   
t = 0
dt = 0.01
desiredAccel = np.array([0, 0, 0], dtype=np.float64)
desiredVel = Vector3(0.0001, 0.0001, 3)

ct = 2
pt = -1

while True:
    payloadVel = Vector3(s0[6], s0[7], s0[8])
        
    if (t - pt >= ct):
        desiredVel = Vector3(gen(), gen(), (gen()/6))
        print(desiredVel)
        pt = t
    
    if t >= 60:
        print("Finished")
        break
    
    desiredAccel = desiredVel.cpy().add(payloadVel.cpy().scl(-1)).cpy().scl(2).arr()

    res = solve_ivp(state_dot, [0, dt], s0, rtol=1e-9, atol=1e-9)
    
    pos = np.array([res.y[0][-1], res.y[1][-1], res.y[2][-1]], dtype=np.float64)
    dpos = np.array([res.y[3][-1], res.y[4][-1], res.y[5][-1]], dtype=np.float64)
    vel = np.array([res.y[6][-1], res.y[7][-1], res.y[8][-1]], dtype=np.float64)
    dvel = np.array([res.y[9][-1], res.y[10][-1], res.y[11][-1]], dtype=np.float64)
    
    s0 = [*pos] + [*dpos] + [*vel] + [*dvel]
    
    d["payload"].append(pos.tolist() + storage.trackedForce.tolist())
    for i in range(4):
        d["drone" + str(i+1)].append((dpos + droneOffsets[i]).tolist())

    t += dt
    
with open('data-rand.json', 'w+') as fout:
    dump(d, fout)

