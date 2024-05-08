import pybullet as p
import pybullet_data
import numpy as np
from time import sleep

def run_simulation(params):
    physicsClient = p.connect(p.GUI)
    timestep = 1./240.
    p.setTimeStep(timestep)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    elevatorId = p.loadURDF("elevator.urdf", [0,0,1], useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
    leverId = p.loadURDF("lever.urdf", [1,0,1.3], p.getQuaternionFromEuler([0, 0, np.pi/2]), useFixedBase=1)
    springBaseCor = p.getLinkState(elevatorId, 4)[0]
    springId = p.createConstraint(elevatorId, 4, -1, -1, p.JOINT_POINT2POINT, [0,0,0], [0,0,0], springBaseCor)
    kId = p.addUserDebugParameter("k",0,100,50)
    cId = p.addUserDebugParameter("c",0,2,0.5)
    leverCor = p.getLinkState(leverId, 1)[0]
    elevCor = p.getLinkState(elevatorId, 3)[4]
    cableL = np.linalg.norm(np.array(leverCor)-np.array(elevCor))
    cableId = p.createConstraint(elevatorId, 3, leverId, 1, p.JOINT_POINT2POINT, [0,0,0], [cableL,0,0.25], [0, 0, 0])
    orn = p.getLinkState(elevatorId, 2)[0]
    stabElevConId = p.createConstraint(elevatorId, 2, -1, -1, p.JOINT_PRISMATIC, [1,0,0], [0,0,0], orn)
    p.setGravity(0,0,-10)
    jointIndexes = [index for index in range(p.getNumJoints(elevatorId))]
    jointForces = [0 for joint in jointIndexes]
    p.setJointMotorControlArray(elevatorId, jointIndexes, p.VELOCITY_CONTROL, forces=jointForces)
    p.setJointMotorControl2(leverId,0,p.VELOCITY_CONTROL,force=0)
    p.enableJointForceTorqueSensor(leverId, 0, enableSensor=1)
    prevL = 0
    cablePrevDifL = 0
    while p.isConnected():
        params["f"]=p.getJointState(leverId, 0)[3]
        p.setJointMotorControl2(leverId,0,p.POSITION_CONTROL,targetPosition=params["angle"])
        k = p.readUserDebugParameter(kId)
        c = p.readUserDebugParameter(cId)
        curSpringCor = p.getLinkState(elevatorId, 4)[0]
        curL = np.linalg.norm(np.array(curSpringCor)-np.array(springBaseCor))
        leverCor = p.getLinkState(leverId, 1)[0]
        elevCor = p.getLinkState(elevatorId, 3)[4]
        cableCurL = np.linalg.norm(np.array(leverCor)-np.array(elevCor))
        cableDifL = cableCurL-cableL
        difL = curL-prevL
        p.changeConstraint(cableId,maxForce=np.abs(-1000*cableDifL-0.1*(cableDifL-cablePrevDifL)/(timestep)))
        p.changeConstraint(springId,maxForce=np.abs(-k*curL-c*difL/(timestep)))
        p.stepSimulation()
        prevL = curL
        cablePrevDifL = cableDifL
        sleep(timestep)