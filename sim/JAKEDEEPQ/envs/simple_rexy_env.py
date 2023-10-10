#needs heavy mods

import gym
import numpy as np
import math
import pybullet as p
import matplotlib.pyplot as plt

class SimpleRexyEnv(gym.Env):
    metadata = {'render.modes': ['human']}  
  
    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=np.array([30,30,25,20,20,100], dtype=np.float32), # LOW VALUES FOR EACH SERVO
            high=np.array([210,150,160,220,178,230], dtype=np.float32)) #HIGH VALUES FOR EACH SERVO 
        self.observation_space = gym.spaces.box.Box(
            low=np.array([-10, -10, -1, -1, -5, -5, 0, 0]),
            high=np.array([10, 10, 1, 1, 5, 5, 5, 5]))
        # consisting of xy position of the car, unit xy orientation of the car, xy velocity of the car, and xy position of a target we want to reach.
        self.np_random, _ = gym.utils.seeding.np_random()

        self.client = p.connect(p.DIRECT) # NOTE : GUI OR NO
        # Reduce length of episodes for RL algorithms
        p.setTimeStep(1/30, self.client)


    def step(self, action):
        pass

    def reset(self):
        pass

    def render(self):
        pass
    
    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
    
    def close(self):
        p.disconnect(self.client)