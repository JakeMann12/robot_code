import pybullet as p
import numpy as np
import time
import pybullet_data
supermaxdist = 0
super_best_params = []
good_motors = [2,4,6,10,12, 14]
 #random initial values
 
for overall_run in range(40):
    print("OUTER RUN NUMBER: ", overall_run)
    #for each run, generate a set of starting parameters
    super_params = [] #rnadom params for this round
    maxdist =0
    best_params = []
    c_val =0.005
    
    #generate random parameters w a+b*np.sin(i*c)
    for i in good_motors:
        my_params = [0,0,0]
        for j in range(1):
            my_params[0]=(np.random.uniform(0,1))#is this supposed to be within range 0-1??
            my_params[1]=(np.random.uniform(0,1))
            my_params[2]=(c_val)
        super_params.append(my_params)
        
    if len(best_params)==0:
        best_params = super_params
        super_best_params = super_params
        
        
    #run 200  times, adjust by percentages at each point
    for inner_run in range(100):
        print("INNER RUN NUMBER: ", inner_run)
        physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.81)
        groundId = p.loadURDF("plane.urdf")
        robotStartPos = [0,0,0.18]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
        robotId = p.loadURDF(r"URDFs\feetasrigidjts\jake.urdf", robotStartPos, robotStartOrientation)
        mode = p.POSITION_CONTROL
        time.sleep(0.05)
        

        for i in range (10000):
            p.stepSimulation()
            for j in range(6):
                a,b,c = super_params[j][0],super_params[j][1],super_params[j][2]
                p.setJointMotorControl2(robotId, good_motors[j], controlMode=mode, targetPosition=a+b*np.sin(i*c))
            
        robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
        new_dist = np.sqrt(robotPos[0]**2 + robotPos[1]**2 + robotPos[2]**2)
        
        #analyze if best so far
        if new_dist > maxdist:
            maxdist = new_dist
            best_params = super_params
        if new_dist > supermaxdist:
            supermaxdist = new_dist
            super_best_params = super_params
            
        #adjust current best by small amount
        alterations_joint = [np.random.randint(0,6) for j in range(5)]
        alterations_param = [np.random.randint(0,3) for j in range(5)]
        super_params = np.copy(best_params) #we only care if this gives us the best version
        
        #alter values
        for change in range(5):
            abc = alterations_param[change]
            value = best_params[alterations_joint[change]][abc]
            value = (1 + np.random.choice([-0.3,0.3]))*(value)
            super_params[alterations_joint[change]][abc] = value
        print("current best distance: ", maxdist)
        p.disconnect()
        
        
        if inner_run >= 50 and maxdist < 0.5:
            print("for this round: ", best_params, maxdist)
            break
        if inner_run >= 100 and maxdist < 1.5:
            print("for this round: ", best_params, maxdist)
            break
        if inner_run >= 200 and maxdist < 2.5:
            print("for this round: ", best_params, maxdist)
            break
    print("for this round: ", best_params, maxdist)
print(super_best_params, supermaxdist)


#%%
#for this round:  [[0.70312131 0.04431859 0.005915  ]
# [0.26623661 0.57702719 0.003185  ]
# [0.07756777 0.91296144 0.0041405 ]
# [0.34295338 1.31905742 0.00455   ]
# [0.87986487 2.08979675 0.0076895 ]
# [0.28227091 1.16768994 0.00455   ]] 0.8803344409312753