import pybullet as p
import os
import math

#%% REQUIRES HEAVY EDITING

class Rexy:
    def __init__(self, client):
        self.client = client
        f_name = r"C:\Users\jmann\Box\Dook Work\Robot Learning\robot_code\URDFs\feetasrigidjts\jake.urdf"
        self.rexy = p.loadURDF(fileName=f_name,
                              basePosition=[0, 0, 0.18], #hardcoded
                              physicsClientId=client,
                              flags=p.URDF_USE_INERTIA_FROM_FILE) #NOTE: added this in extra from hello_bullet
        self.servo_joints = [2, 4, 6, 10, 12, 14]

    def get_ids(self):
        return self.client, self.rexy

    def apply_action(self, action):
        pass
    
    def get_observation(self):
        # Get the position and orientation of the rexy in the simulation
        pos, ang = p.getBasePositionAndOrientation(self.rexy, self.client)
        ang = p.getEulerFromQuaternion(ang)
        ori = (math.cos(ang[2]), math.sin(ang[2]))
        pos = pos[:2]
        # Get the velocity of the rexy
        vel = p.getBaseVelocity(self.rexy, self.client)[0][0:2]

        # Concatenate position, orientation, velocity
        observation = (pos + ori + vel)

        return observation

    









