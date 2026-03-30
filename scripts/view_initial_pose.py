#!/usr/bin/env python
'''Observe the initial state of the robot

Usage:
    python scripts/view_initial_pose.py
'''

import numpy as np
from envs.jvrc.jvrc_walk import JvrcWalkEnv
import time
import mujoco

def main():
    env = JvrcWalkEnv()
    env.reset()
    env.render()

    while env.viewer.is_running():
        env.render()
        time.sleep(0.01)


if __name__ == "__main__":
    main()