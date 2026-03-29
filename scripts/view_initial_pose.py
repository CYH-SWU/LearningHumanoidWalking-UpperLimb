'''Observe the initial state of the robot

Usage:
    python scripts/view_initial_pose.py
'''

import time
import mujoco
from envs.jvrc.jvrc_walk import JvrcWalkEnv


def main():
    env = JvrcWalkEnv()
    env.reset()
    env.render()

    while env.viewer.is_running():
        env.render()
        time.sleep(0.01)

    print("---------Open---------")

    

    env.close()
    print("Exit !!!")

if __name__ == "__main__":
    main()