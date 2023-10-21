import pybullet as p
import time
import numpy as np
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
robotStartPos = [0,0,0.18]
robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
mode = p.POSITION_CONTROL
robotId = p.loadURDF(r"C:\Users\jmann\Box\Dook Work\Robot Learning\robot_code\URDFs\feetasrigidjts\jake.urdf", robotStartPos, robotStartOrientation)
joints = [2,4,6,10,12, 14]
time.sleep(1)
#best params from hillclimber:
#   [[1.4590984345374207, 0.461585909369459, 0.045346267881847924]
#    [1.3759019682649325, 0.9924465584425969, 0.045346267881847924], 
#    [1.0057676976891672, 0.361584453701813, 0.045346267881847924], 
#    [0.23100155405539002, 0.2907837369258507, 0.045346267881847924], 
#    [0.7106237891034008, 0.23337288085082375, 0.045346267881847924], 
#    [1.7611323145031526, 0.42524171715505366, 0.045346267881847924]] 
# dist: 3.358112673468683
best_params = [[-1.47158792e+00, -2.70801203e-01,  1.15126471e-03,  8.91459206e-01],
 [-8.12680319e-02, -2.11443201e-01,  1.15126471e-03,  9.05724989e-02],
 [ 2.00944000e+00,  3.61630444e-01,  1.15126471e-03,-4.22120107e-02],
 [ 3.13782497e-02, -1.83845335e+00,  1.15126471e-03,  9.92570684e-01],
 [-7.47734788e-03, -4.15091371e-03,  1.15126471e-03, -4.53312399e-01],
 [-3.14138753e-02,  1.29051500e-01,  1.15126471e-03,  2.03768541e-01]]

 #x = current position of joint 0
 #x = a+bsin(ci + d)
 # so (x-a)/b = sin(ic)
 #arcsin((x-a)/b)=ic
 # i = arcsin((x-a)/b)*(1/c)
for i in joints:
    print(p.getJointInfo(robotId, i))
i=0
# for i in range (5000):
#set initial positions:
#solve for initial position:
for i in range(24):
    print(p.getJointInfo(robotId, i))

while i < 5000:
    p.stepSimulation()
    for j in range(6):
        a,b,c,d= best_params[j][0],best_params[j][1],best_params[j][2],best_params[j][3]
        p.setJointMotorControl2(robotId, joints[j], controlMode=mode, targetPosition=a+b*np.sin(10*i*c+d))
    i +=0.1
# for i in range(5000):
#     p.stepSimulation()
#     for j in range(len(joints)):
#         p.setJointMotorControl2(robotId, joints[j], controlMode=mode, targetPosition=best_params[j][0]+best_params[j][1]*np.sin(i*best_params[j][2]))
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()
