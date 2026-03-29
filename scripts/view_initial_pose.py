'''Observe the initial state of the robot

Usage:
    python scripts/view_initial_pose.py
'''

import time
from envs.jvrc.jvrc_walk import JvrcWalkEnv
import numpy as np

def main():
    env = JvrcWalkEnv()
    env.reset()
    env.render()
    
    while env.viewer.is_running():
        env.render()
        time.sleep(0.01)
        # env.step(np.zeros(env.action_space.shape[0]))
    env.close()

if __name__ == "__main__":
    main()
    