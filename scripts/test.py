#!/usr/bin/env python
'''Send sinusoidal commands to all major joints (legs and arms) to test motion range and PD control.

Usage:
    python scripts/test.py
'''

import time
import numpy as np
from envs.jvrc.jvrc_walk import JvrcWalkEnv

def main():
    env = JvrcWalkEnv()
    env.reset()
    env.model.opt.gravity[2] = 0.0
    env.render()

    # Control period from config
    dt = env.cfg.control_dt   # 0.025 s
    # Neutral joint angles (excluding base position and orientation)
    neutral_joints = np.array(env.nominal_pose[7:])  # shape (16,)

    t = 0.0
    freq = 0.5          # oscillation frequency (Hz)
    amplitude = 0.6     # amplitude for hip (rad)

    # Joint indices based on actuator order:
    # 0:R_HIP_P, 1:R_HIP_R, 2:R_HIP_Y, 3:R_KNEE, 4:R_ANKLE_R, 5:R_ANKLE_P,
    # 6:L_HIP_P, 7:L_HIP_R, 8:L_HIP_Y, 9:L_KNEE, 10:L_ANKLE_R, 11:L_ANKLE_P,
    # 12:R_SHOULDER_P, 13:R_ELBOW_P, 14:L_SHOULDER_P, 15:L_ELBOW_P

    while env.viewer.is_running():
        offset = amplitude * np.sin(2 * np.pi * freq * t)

        action = neutral_joints.copy()

        # Legs: hip pitch, knee, ankle pitch
        action[0] += offset          # R_HIP_P
        action[6] += offset          # L_HIP_P
        action[3] += offset          # R_KNEE
        action[9] += offset          # L_KNEE
        action[5] += offset          # R_ANKLE_P
        action[11] += offset         # L_ANKLE_P

        # Arms: shoulder pitch and elbow pitch (in-phase with legs)
        action[12] += offset         # R_SHOULDER_P
        action[14] += offset         # L_SHOULDER_P
        action[13] += offset         # R_ELBOW_P (half amplitude)
        action[15] += offset         # L_ELBOW_P

        env.step(action)
        env.render()
        time.sleep(dt)
        t += dt

if __name__ == "__main__":
    main()