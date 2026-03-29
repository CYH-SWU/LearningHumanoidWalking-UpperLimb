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
    # 在 env = JvrcStepEnv() 之后添加
    print("\n实际执行器顺序（关节名称）:")
    joint_names = env.interface.get_actuated_joint_names()
    for i, name in enumerate(joint_names):
        print(f"  actuator {i}: {name}")
    env.reset()
    env.render()

    while env.viewer.is_running():
        env.step(np.zeros(env.action_space.shape[0]))
        env.render()
        time.sleep(0.01)


if __name__ == "__main__":
    main()