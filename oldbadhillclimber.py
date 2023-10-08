import pybullet as p
import numpy as np
import time
import pybullet_data
#sdihfgasdlifh
supermaxdist = 0
maxdist =0
best_params = []
super_best_params = []
 #random initial values

for overall_run in range(20):
    print("OUTER RUN NUMBER: ", overall_run)
    #for each run, generate a set of starting parameters
    super_params = []
    for i in range(7):
        #asin(bx)+c
        my_params = []
        for j in range(3):
            my_params.append(np.random.random()) #is this supposed to be within range 0-1??
        super_params.append(my_params)
    #run 20  times, adjust by percentages at each point
    for inner_run in range(20):
        print("INNER RUN NUMBER: ", inner_run)
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        robotStartPos = [0,0,1]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
        robotId = p.loadURDF("jake.urdf", robotStartPos, robotStartOrientation)
        mode = p.POSITION_CONTROL
        maxforce = 500
        jointIndex = 0 

        for i in range (10000):
            p.stepSimulation()
            for j in range(7):
                joint_params = super_params[j]
                a,b,c = joint_params[0],joint_params[1],joint_params[2]
                p.setJointMotorControl2(robotId, j, controlMode=mode, targetPosition=a+b*np.sin(i*c))
            time.sleep(1./240.)
        robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
        new_dist = np.sqrt(robotPos[0]**2 + robotPos[1]**2 + robotPos[2]**2)
        if new_dist > maxdist:
            maxdist = new_dist
            best_params = super_params
        if new_dist > supermaxdist:
            supermaxdist = new_dist
            super_best_params = super_params
        alterations_joint = [np.random.randint(0,7) for j in range(5)]
        alterations_param = [np.random.randint(0,3) for j in range(5)]
        super_params = np.copy(best_params)
        for change in range(5):
            abc = alterations_param[change]
            value = best_params[alterations_joint[change]][abc]
            value = (1 + np.random.uniform(-0.1,0.1))*(value)
            super_params[alterations_joint[change]][abc] = value
        # print(robotPos, robotOrn)
        p.disconnect()
