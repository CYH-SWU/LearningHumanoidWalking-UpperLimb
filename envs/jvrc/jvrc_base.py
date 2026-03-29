"""Base class for JVRC robot environments.

This module provides common functionality shared across JVRC environment
implementations (walking, stepping, etc.).
"""

import os
from abc import abstractmethod

import numpy as np

from envs.common import robot_interface
from envs.common.base_humanoid_env import BaseHumanoidEnv
from robots.robot_base import RobotBase
from tasks import observations as obs

from .gen_xml import LEG_JOINTS, KEEP_ARM_JOINTS


class JvrcBaseEnv(BaseHumanoidEnv):
    """Base class for JVRC humanoid environments.

    Provides JVRC-specific common functionality:
    - Nominal pose and interface setup
    - PD gains configuration
    - Mirror indices for symmetry learning
    """

    # JVRC body names
    RFOOT_BODY = "R_ANKLE_P_S"
    LFOOT_BODY = "L_ANKLE_P_S"
    ROOT_BODY = "PELVIS_S"
    HEAD_BODY = "NECK_P_S"

    def _get_default_config_path(self) -> str:
        return os.path.join(os.path.dirname(os.path.realpath(__file__)), "configs/base.yaml")

    def _setup_robot(self) -> None:
        control_dt = self.cfg.control_dt

        self.actuators = LEG_JOINTS + KEEP_ARM_JOINTS
        print("=== actuators order ===")
        print("Legs (first 12):", self.actuators[:12])
        print("Arms (last 4):", self.actuators[12:])

        # PD gains from config
        total_act = len(self.actuators)
        pdgains = np.zeros((2, total_act))
        # Legs
        pdgains[0, :12] = self.cfg.kp
        pdgains[1, :12] = self.cfg.kd
        # Arms
        arm_kp = getattr(self.cfg, "arm_kp", [0.5] * 4)
        arm_kd = getattr(self.cfg, "arm_kd", [0.05] * 4)
        pdgains[0, 12:] = arm_kp
        pdgains[1, 12:] = arm_kd
        

        # Get half-sitting pose from config (in degrees)
        self.half_sitting_pose = self.cfg.half_sitting_pose
        arm_angles_deg = getattr(self.cfg, "arm_half_sitting_pose", [0, 0, 0, 0])
        arm_angles_rad = np.deg2rad(arm_angles_deg).tolist()

        # Define nominal pose
        base_position = [0, 0, 0.81]
        base_orientation = [1, 0, 0, 0]
        self.nominal_pose = base_position + base_orientation + np.deg2rad(self.half_sitting_pose).tolist()\
        + arm_angles_rad

        # Setup interface
        self.interface = robot_interface.RobotInterface(self.model, self.data, self.RFOOT_BODY, self.LFOOT_BODY, None)

        # Setup task (implemented by subclasses)
        self._setup_task(control_dt)

        # Setup robot
        self.robot = RobotBase(pdgains, control_dt, self.interface, self.task)

        # Setup mirror indices for symmetry learning
        self._setup_mirror_indices()

    @abstractmethod
    def _setup_task(self, control_dt: float) -> None:
        """Setup the task instance. Must set self.task."""
        pass

    def _setup_mirror_indices(self) -> None:
        """Setup mirror indices for symmetry-based learning."""
        base_mir_obs = [
            -0.1,
            1, -2, 3, -4,
            11, -12, -13, 14, -15, 16,
            5, -6, -7, 8, -9, 10,
            23, -24, -25, 26, -27, 28,
            17, -18, -19, 20, -21, 22
        ]
        current_len = len(base_mir_obs)

        base_mir_obs.extend([
            -(current_len + 2),
            -(current_len + 3),
            -(current_len + 0),
            -(current_len + 1),
        ])

        base_mir_obs.extend([
            -(current_len + 6),
            -(current_len + 7),
            -(current_len + 4),
            -(current_len + 5),
        ])

        num_ext_obs = self._get_num_external_obs()
        append_obs = [(len(base_mir_obs) + i) for i in range(num_ext_obs)]
        self.robot.clock_inds = append_obs[0:2]
        self.robot.mirrored_obs = np.array(base_mir_obs + append_obs, copy=True).tolist()

        self.robot.mirrored_acts = [
            5, -6, -7, 8, -9, 10, 0.1, -0, -1, 2, -3, 4,
            -14, -15, -12, -13
        ]

    @abstractmethod
    def _get_num_external_obs(self) -> int:
        """Return the number of external observation dimensions."""
        pass

    def _setup_spaces(self) -> None:
        action_space_size = len(self.actuators)
        self.action_space = np.zeros(action_space_size)
        self.prev_prediction = np.zeros(action_space_size)

        self.base_obs_len = len(self._get_robot_state()) + self._get_num_external_obs()  # get state of robot
        self.observation_space = np.zeros(self.base_obs_len * self.history_len)

        # Setup observation normalization
        self._setup_obs_normalization()

    @abstractmethod
    def _setup_obs_normalization(self) -> None:
        """Setup obs_mean and obs_std for observation normalization."""
        pass

    def _get_robot_state(self) -> np.ndarray:
        root_r, root_p = obs.get_root_orientation(self.interface)
        root_ang_vel = obs.get_root_angular_velocity(self.interface)
        motor_pos = obs.get_motor_positions(self.interface)
        motor_vel = obs.get_motor_velocities(self.interface)
        return np.concatenate([root_r, root_p, root_ang_vel, motor_pos, motor_vel])

    def _get_clock_signal(self) -> list:
        """Get the phase clock signal."""
        return [
            np.sin(2 * np.pi * self.task._phase / self.task._period),
            np.cos(2 * np.pi * self.task._phase / self.task._period),
        ]
