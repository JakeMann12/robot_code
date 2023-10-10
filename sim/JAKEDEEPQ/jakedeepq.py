import pybullet as p
import time
import pybullet_data

#%% Load into pybullet!
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
StartPos = [0,0,0.18]
StartOrientation = p.getQuaternionFromEuler([0,0,0])
rexy = p.loadURDF(r"C:\Users\jmann\Box\Dook Work\Robot Learning\robot_code\URDFs\feetasrigidjts\jake.urdf",StartPos, StartOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE)


#%% early joint calibration efforts
number_of_joints = p.getNumJoints(rexy)
print("NO. OF JOINTS : "+str(number_of_joints)) 
for joint_number in range(number_of_joints):
    info = p.getJointInfo(rexy, joint_number)
    print(info[0], ": ", info[1]) #tuple of joint data - 25 total but that includes rigid

#%%BE A BIG STEPPER

for i in range (10000):
    pos, ori = p.getBasePositionAndOrientation(rexy)
    p.applyExternalForce(rexy, 0, [0, 0, 0], pos, p.WORLD_FRAME) # 0 EXTERNAL FORCE- FUN TEST
    p.stepSimulation()
    if i //100 == i / 100:
        print(pos,ori)
    time.sleep(1./240.)
Pos, Orn = p.getBasePositionAndOrientation(rexy)



print(Pos,Orn)
# FROM THIS, WE'VE LEARNED THAT WE WANT THE Z COORD TO BE MORE THAN ~.2 AT ALL TIMES- ELSE, FALL
p.disconnect()











#%% JOINT LIMITS

def stand(servo1,servo2,servo3,servo4,servo5,servo6):
    
    servo1.set_angle_offset(-30.00) # NOTE : tf?


    servo1.set_angle_limits(0, 180)
    servo2.set_angle_limits(30, 150)
    servo3.set_angle_limits(25, 160)
    servo4.set_angle_limits(20, 220)
    servo5.set_angle_limits(20, 178)
    servo6.set_angle_limits(100, 230)
    
    servo1.move(32.88)
    servo2.move(94.32)
    servo3.move(50)
    servo4.move(147)
    servo5.move(98.16)
    servo6.move(136)