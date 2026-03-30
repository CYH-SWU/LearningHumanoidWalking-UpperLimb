# LearningHumanoidWalking – Upper Limb Edition

> This is a modified version of [rohanpsingh/LearningHumanoidWalking](https://github.com/rohanpsingh/LearningHumanoidWalking) that adds upper‑limb control for the JVRC‑1 humanoid robot.  
> For installation, training, and evaluation instructions, please refer to the original repository.

## Our Modifications

We extended the original project to incorporate upper‑limb control. Key changes:

### 1. Model Construction (`envs/jvrc/gen_xml.py`)
- Retained shoulder pitch and elbow pitch DOFs (4 joints). Removed all other upper‑limb joints.
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

### 4. Reward Functions (`tasks/rewards.py`, `tasks/stepping_task.py`)
Added three upper‑limb specific rewards (weights adjustable):
- **Motion Encouragement**: penalizes static arms.
- **Phase Matching**: rewards arm swing opposite to contralateral leg.
- **Energy Penalty**: discourages high‑frequency oscillations.

### 5. Configuration (`envs/jvrc/configs/base.yaml`)
- Added `arm_half_sitting_pose`, `arm_kp`, `arm_kd`.
- Adjusted lower‑limb PD gains for better stability.

### 6. Testing (`scripts/view_initial_pose.py`)
- Script to visually inspect the robot’s initial posture.

### 7. Mirroring Fix (`envs/jvrc/jvrc_base.py`)
- Corrected mirroring logic to match actual joint ordering.

All changes preserve the original project structure, making it easy to switch between versions.

## License & Credits
This project is a derivative of [LearningHumanoidWalking](https://github.com/rohanpsingh/LearningHumanoidWalking).  
Please see the original repository for licensing details and citation information.