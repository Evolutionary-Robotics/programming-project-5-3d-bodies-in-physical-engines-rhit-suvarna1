import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import numpy as np
import time 

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
p.changeDynamics(planeId, -1, lateralFriction=1.0)
p.setGravity(0,0,-9.8)

# p.loadSDF("box.sdf")
# p.loadSDF("box2.sdf")
wormSim = p.loadURDF("worm.urdf")
p.changeDynamics(wormSim, 4, lateralFriction = 10.0)

duration = 10000

joint_lower_limit = 0
joint_upper_limit = np.pi / 4  
back_joint_index = 4
front_joint_index = 0
ps.Prepare_To_Simulate(wormSim)
x = np.linspace(0,40*np.pi, duration)
y = np.sin(x)*np.pi/2
for i in range(duration):

    target_position = y[i]
    
    if target_position < joint_lower_limit:
        target_position = joint_lower_limit
    elif target_position > joint_upper_limit:
        target_position = joint_upper_limit

    # p.setJointMotorControl2(bodyIndex=wormSim,
    #                         jointIndex=0,  # Index of the joint to control (0 for the first joint)
    #                         controlMode=p.POSITION_CONTROL,
    #                         targetPosition=target_position,
    #                         force=500)
    p.setJointMotorControl2(bodyIndex=wormSim,
                            jointIndex=1,  # Index of the joint to control (0 for the first joint)
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=-target_position,
                            force=500)
    p.setJointMotorControl2(bodyIndex=wormSim,
                            jointIndex=2,  # Index of the joint to control (0 for the first joint)
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=target_position,
                            force=500)
    p.setJointMotorControl2(bodyIndex=wormSim,
                            jointIndex=3,  # Index of the joint to control (0 for the first joint)
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=target_position,
                            force=500)
    
    if target_position < joint_upper_limit / 2:
        p.changeDynamics(wormSim, front_joint_index, lateralFriction=10)
        p.changeDynamics(wormSim, back_joint_index, lateralFriction=0.1)  # Reduce friction during scrunch
    else:
        p.changeDynamics(wormSim, back_joint_index, lateralFriction=10.0)  # High friction during extension
        p.changeDynamics(wormSim, back_joint_index, lateralFriction=0.1)

    p.stepSimulation()
    time.sleep(1/500)


p.disconnect()