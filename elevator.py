import pybullet as p
import pybullet_data
import numpy as np
from time import sleep

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
elevatorId = p.loadURDF("elevator.urdf", [0,0,1], useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
leverId = p.loadURDF("lever.urdf", [1,0,1.3], p.getQuaternionFromEuler([0, 0, np.pi/2]), useFixedBase=1)
wx, wy, wz = p.getLinkState(elevatorId, 4)[0]
springId = p.createConstraint(elevatorId, 3, -1, -1, p.JOINT_FIXED, [0,0,0], [0,0,-0.25], [wx, wy, wz])
kId = p.addUserDebugParameter("k",0,1000,500)
cId = p.addUserDebugParameter("c",0,2,0.5)
lx, ly, lz = p.getLinkState(leverId, 1)[0]
ex, ey, ez = p.getLinkState(elevatorId, 3)[0]
cableL = np.sqrt((lx-ex)**2+(ly-ey)**2+(lz-ez)**2)
cableId = p.createConstraint(elevatorId, 3, leverId, 0, p.JOINT_FIXED, [0,0,0], [0,0,0.25], [0, 0, -0.5])
# p.setGravity(0,0,-10)
# start = True
p.setJointMotorControl2(elevatorId,0,p.VELOCITY_CONTROL,force=0)
p.setJointMotorControl2(elevatorId,1,p.VELOCITY_CONTROL,force=0)
p.setJointMotorControl2(elevatorId,2,p.VELOCITY_CONTROL,force=0)
p.setJointMotorControl2(leverId,0,p.VELOCITY_CONTROL,force=0)
p.enableJointForceTorqueSensor(leverId, 0, enableSensor=1)
t=0
prevL = 0
cablePrevDifL = 0
while p.isConnected():
    print(p.getJointState(leverId, 0)[3])
    p.setJointMotorControl2(leverId,0,p.POSITION_CONTROL,targetPosition=0.3*np.sin(2*np.pi*t))
    k = p.readUserDebugParameter(kId)
    c = p.readUserDebugParameter(cId)
    x, y, z = p.getLinkState(elevatorId, 4)[0]
    l = np.sqrt((x-wx)**2+(y-wy)**2+(z-wz)**2)
    curL = l
    lx, ly, lz = p.getLinkState(leverId, 1)[0]
    ex, ey, ez = p.getLinkState(elevatorId, 3)[0]
    cableCurL = np.sqrt((lx-ex)**2+(ly-ey)**2+(lz-ez)**2)
    cableDifL = cableCurL-cableL
    # if start:
    #     start = False
    #     prevL = l
    #     cablePrevDifL = cableDifL
    difL = curL-prevL
    p.changeConstraint(cableId,maxForce=1000*(cableCurL-cableL)+0.1*(cableDifL-cablePrevDifL)/(1./240.))# if cableDifL > 0 else 0)
    p.changeConstraint(springId,maxForce=k*curL+c*difL/(1./240.))
    p.stepSimulation()
    prevL = l
    cablePrevDifL = cableDifL
    t+=1./240.
    sleep(1./240.)