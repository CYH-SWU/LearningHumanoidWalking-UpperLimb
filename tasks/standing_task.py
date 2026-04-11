"""Standing balance task for humanoid robots.

This module defines a simple standing task where the robot must maintain
an upright posture without walking.
"""

import numpy as np

from tasks.base_task import BaseTask


class StandingTask(BaseTask):
    """Standing balance task for humanoid robots.

    The robot must maintain an upright standing posture, minimizing
    velocity, keeping the torso aligned, and maintaining a target height.

    Attributes:
        _client: RobotInterface for accessing robot state.
        neutral_pose: Target joint positions for standing.
    """

    def __init__(self, client, neutral_pose, arm_neutral_pose=None):
        """Initialize the standing task.

        Args:
            client: RobotInterface instance for robot state access.
            neutral_pose: Target joint positions for the standing posture.
        """
        self._client = client
        self.neutral_pose = neutral_pose
        self.arm_neutral_pose = np.array(arm_neutral_pose) if arm_neutral_pose is not None else np.zeros(4)

    def reset(self, iter_count: int = 0) -> None:
        """Reset task state for a new episode.

        Standing task has no state to reset.

        Args:
            iter_count: Current training iteration (unused).
        """
        pass

    def step(self) -> None:
        pass

    def substep(self) -> None:
        pass

    def calc_reward(
        self,
        prev_torque: np.ndarray,
        prev_action: np.ndarray,
        action: np.ndarray,
    ) -> dict[str, float]:
        root_pose = self._client.get_object_affine_by_name("pelvis", "OBJ_BODY")

        # height reward
        target_root_h = 0.98
        root_h = root_pose[2, 3]
        height_error = np.linalg.norm(root_h - target_root_h)

        # upperbody reward
        head_pose_offset = np.zeros(2)
        head_pose = self._client.get_object_affine_by_name("torso_link", "OBJ_BODY")
        head_pos_in_robot_base = np.linalg.inv(root_pose).dot(head_pose)[:2, 3] - head_pose_offset
        upperbody_error = np.linalg.norm(head_pos_in_robot_base)

        # posture reward
        current_pose = np.array(self._client.get_act_joint_positions())[:10]
        posture_error = np.linalg.norm(current_pose - self.neutral_pose)

        # torque reward
        tau_error = np.linalg.norm(self._client.get_act_joint_torques())

        # velocity reward
        root_vel = self._client.get_body_vel("pelvis", frame=1)[0][:2]
        fwd_vel_error = np.linalg.norm(root_vel)
        yaw_vel = self._client.get_qvel()[5]
        yaw_vel_error = np.linalg.norm(yaw_vel)

        # Upper‑limb states (last 4 joints: R_shoulder, L_shoulder, R_elbow, L_elbow)
        arm_pos = self._client.get_act_joint_positions()[-4:]
        arm_vel = self._client.get_act_joint_velocities()[-4:]
        arm_torque = self._client.get_act_joint_torques()[-4:]

        # Upper‑limb reward: target micro‑bent pose (from config)
        arm_neutral = self.arm_neutral_pose
        arm_posture_error = np.linalg.norm(arm_pos - arm_neutral)
        arm_posture_reward = 0.05 * np.exp(-20 * arm_posture_error)

        # Penalize large torques
        arm_torque_penalty = -0.0005 * np.sum(np.square(arm_torque))

        # Slight bonus for natural motion (peak at 0.1 rad/s)
        arm_speed = np.mean(np.abs(arm_vel))
        arm_motion_bonus = 0.01 * np.exp(-5 * (arm_speed - 0.1)**2)

        reward = {
            "com_vel_error": 0.3 * np.exp(-4 * np.square(fwd_vel_error)),
            "yaw_vel_error": 0.3 * np.exp(-4 * np.square(yaw_vel_error)),
            "height": 0.1 * np.exp(-0.5 * np.square(height_error)),
            "upperbody": 0.1 * np.exp(-40 * np.square(upperbody_error)),
            "joint_torque_reward": 0.1 * np.exp(-5e-5 * np.square(tau_error)),
            "posture": 0.1 * np.exp(-1 * np.square(posture_error)),
            # Upper‑limb components
            "arm_posture": arm_posture_reward,
            "arm_torque_penalty": arm_torque_penalty,
            "arm_motion_bonus": arm_motion_bonus,
        }
        return reward

    def done(self) -> bool:
        """Check if the episode should terminate.

        Terminates if:
        - Root height drops below 0.9m (falling)
        - Root height exceeds 1.4m (unrealistic)
        - Self-collision detected

        Returns:
            True if episode should terminate, False otherwise.
        """
        root_jnt_adr = self._client.model.body("pelvis").jntadr[0]
        root_qpos_adr = self._client.model.joint(root_jnt_adr).qposadr[0]
        qpos = self._client.get_qpos()[root_qpos_adr : root_qpos_adr + 7]
        contact_flag = self._client.check_self_collisions()

        terminate_conditions = {
            "qpos[2]_ll": (qpos[2] < 0.9),
            "qpos[2]_ul": (qpos[2] > 1.4),
            "contact_flag": contact_flag,
        }

        return True in terminate_conditions.values()
