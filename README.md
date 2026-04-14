# LearningHumanoidWalking – Upper Limb & Speed Optimization Edition

> This is an extended fork of [rohanpsingh/LearningHumanoidWalking](https://github.com/rohanpsingh/LearningHumanoidWalking) that adds upper‑limb control and high‑speed walking capabilities to the JVRC‑1 humanoid robot.  
> For installation, training, and evaluation instructions, please refer to the original repository.

## Our Modifications

We extended the original project with two major improvements: upper‑limb integration and stable fast walking. Key changes are listed below.

### 1. Model Construction (`envs/jvrc/gen_xml.py`)
- Retained shoulder pitch and elbow pitch DOFs (4 joints); removed all other upper‑limb joints.
- Set joint limits: shoulder ±45°, elbow –90° to 0°.
- Added capsule collision shapes for upper arm and forearm with contact filtering.
- Configured separate torque limits for lower and upper limbs.

### 2. Environment Base (`envs/jvrc/jvrc_base.py`)
- Extended action space from 12 to 16 dimensions.
- Built separate PD gain matrices for lower and upper limbs.
- Updated `nominal_pose` to include initial upper‑limb angles.
- Fixed mirror mapping for observations and actions to ensure proper symmetry.

### 3. Subclass Adaptations (`envs/jvrc/jvrc_step.py`, `envs/jvrc/jvrc_walk.py`)
- Added mean and std for new upper‑limb joints in observation normalization.
- **New observation:** `self._goal_speed_ref` (target forward speed) is now part of the external state.

### 4. Reward Functions (`tasks/rewards.py`, `tasks/stepping_task.py`, `tasks/walking_task`)
- **Upper‑limb rewards** (weights adjustable):
  - Motion Encouragement: penalizes static arms.
  - Phase Matching: rewards arm swing opposite to contralateral leg.
  - Energy Penalty: discourages high‑frequency oscillations.
- **Velocity tracking reward** (new):
  - Encourages the robot to reach a target forward speed (`self._goal_speed_ref`).
  - Implemented as `vel_reward = exp(-3 * (actual_vel - target_vel)^2)`.

### 5. Fast Walking Tuning
- **Gait parameters**:
  - Step cycle: `total_duration = 1.0 s`
  - Swing phase: `swing_duration = 0.7 s`
  - Stance phase: `stance_duration = 0.3 s`
- **Step length**: fixed to `0.325 m` (theoretically yields 0.65 m/s with 1.0 s cycle).
- **Target speed**: set to `0.65 m/s` (constant, or sampled in a narrow range).
- **Velocity reward weight**: `0.25` (balanced with other terms).
- These changes result in **more stable and faster walking** compared to the original.

### 6. Configuration (`envs/jvrc/configs/base.yaml`)
- Added `arm_half_sitting_pose`, `arm_kp`, `arm_kd`.
- Adjusted lower‑limb PD gains for better stability.

### 7. Testing (`scripts/view_initial_pose.py`)
- Script to visually inspect the robot’s initial posture.

### 8. Mirroring Fix (`envs/jvrc/jvrc_base.py`)
- Corrected mirroring logic to match actual joint ordering.

### 9. Sinusoidal commands (`scripts/test.py`)
- Send sinusoidal commands to all major joints (legs and arms) to test motion range and PD control.

All changes preserve the original project structure, making it easy to switch between versions.

## License & Credits
This project is a derivative of [LearningHumanoidWalking](https://github.com/rohanpsingh/LearningHumanoidWalking).  
Please see the original repository for licensing details and citation information.