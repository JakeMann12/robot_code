import gym
import torch
from agent import TRPOAgent
import simple_rexy.envs.simple_rexy_env
import time

#zswang666 and electricelephant

from gym.envs.registration import register
register(
    id='SimpleRexyEnv-v0',
    entry_point='simple_rexy.envs:simple_rexy_env'
)


def main():
    nn = torch.nn.Sequential(torch.nn.Linear(8, 64), torch.nn.Tanh(),
                             torch.nn.Linear(64, 2))
    agent = TRPOAgent(policy=nn)

    #agent.load_model("agent.pth")
    agent.train("SimpleRexyEnv-v0", seed=0, batch_size=5000, iterations=100,
                max_episode_length=250, verbose=True)
    agent.save_model("agent.pth")

    env = gym.make('SimpleRexyEnv-v0')
    ob = env.reset()
    while True:
        action = agent(ob)
        ob, _, done, _ = env.step(action)
        env.render()
        if done:
            ob = env.reset()
            time.sleep(1/30)


if __name__ == '__main__':
    main()
