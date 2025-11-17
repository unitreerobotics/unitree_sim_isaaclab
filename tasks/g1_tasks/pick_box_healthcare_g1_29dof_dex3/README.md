# Healthcare Box Picking Task - G1 Robot with Dex3 Hands

## Overview

This task implements a box picking scenario in a healthcare environment using the Unitree G1 humanoid robot (29 DOF) with Dex3 dexterous hands. The robot must pick up a small box from a workspace and lift it to a target height.

## Task Details

### Environment
- **Scene**: Healthcare environment from Omniverse
  - USD Path: `omniverse://isaac-dev.ov.nvidia.com/Library/IsaacHealthcare/0.5.0/Props/OrcaScenes/Scene1/main.usd`
- **Robot**: Unitree G1 with wholebody control
  - 29 DOF: 12 leg joints + 3 waist joints + 14 arm joints
  - Dex3 dexterous hands
  - Initial position: `(0.0, -1.0, 0.0)` - in front of workspace

### Object
- **Type**: Cubic box
- **Dimensions**: 6cm x 6cm x 6cm
- **Mass**: 200 grams
- **Color**: Orange/brown
- **Initial Position**: `(0.5, 0.0, 0.85)` - on workspace surface
- **Physics**: High friction material for stable grasping

### Observations

The policy receives the following observations (not concatenated):

1. **`robot_joint_state`** (87 values):
   - 29 joint positions
   - 29 joint velocities
   - 29 joint torques

2. **`robot_gripper_state`**:
   - Dex3 hand joint states (positions, velocities, torques)

3. **`camera_image`**:
   - Front camera (robot head)
   - Left wrist camera
   - Right wrist camera
   - World camera (external view)

4. **`object_state`** (13 values):
   - Position (3): xyz coordinates
   - Orientation (4): quaternion (w,x,y,z)
   - Linear velocity (3): xyz velocities
   - Angular velocity (3): xyz angular velocities

### Actions

- **Type**: Direct joint position control
- **Control**: All 29 body DOF + Dex3 finger joints
- **Control Frequency**: 50 Hz (decimation = 4)

### Rewards

**Main Reward** (`picking_reward`, weight=1.0):
- Height reward: Proportional to box height above ground
- Success bonus: +2.0 for lifting box above 0.9m
- Stability bonus: Exponential reward for keeping box stable (low velocity)
- Workspace penalty: -1.0 if box moves outside workspace

**Drop Penalty** (`drop_penalty`, weight=-2.0):
- -1.0 penalty if box drops below 0.5m height

### Terminations

- **`object_out_of_bounds`**: Episode ends if box:
  - Falls below 0.3m height
  - Rises above 2.0m height
  - Moves outside workspace bounds: x ∈ [-2, 2], y ∈ [-2, 2]

- **Episode Length**: 30 seconds (1500 steps at 50 Hz)

### Success Criteria

The task is considered successful when the box is:
- Lifted above 0.9m height
- Maintained stable (low velocity)
- Kept within workspace boundaries

## Usage

### Register and Create Environment

```python
import gymnasium as gym

# Create the environment
env = gym.make("Isaac-Pick-Box-Healthcare-G129-Dex3-Wholebody", num_envs=1)

# Reset the environment
obs, info = env.reset()

# Run a step
action = policy.get_action(obs)  # Your policy
obs, reward, terminated, truncated, info = env.step(action)
```

### Run with Isaac Lab

```bash
# Navigate to Isaac Lab directory
cd /home/nvidia/binliu/unitree_sim_isaaclab

# Run the task
python sim_main.py --device cpu --enable_cameras --task Isaac-Pick-Box-Healthcare-G129-Dex3-Wholebody --enable_dex3_dds --robot_type g129
```

### Training Example

```python
from tasks.g1_tasks import pick_box_healthcare_g1_29dof_dex3

# Access the configuration
env_cfg = pick_box_healthcare_g1_29dof_dex3.pick_box_healthcare_g1_29dof_dex3_env_cfg.PickBoxHealthcareG129Dex3EnvCfg()

# Customize parameters
env_cfg.scene.num_envs = 4  # Train with 4 parallel environments
env_cfg.episode_length_s = 40.0  # Longer episodes

# Create environment
import gymnasium as gym
env = gym.make("Isaac-Pick-Box-Healthcare-G129-Dex3-Wholebody", env_cfg=env_cfg)
```

## File Structure

```
pick_box_healthcare_g1_29dof_dex3/
├── __init__.py                                 # Gym registration
├── pick_box_healthcare_g1_29dof_dex3_env_cfg.py  # Main environment config
├── README.md                                   # This file
└── mdp/
    ├── __init__.py                            # MDP module exports
    ├── observations.py                        # Observation functions
    ├── rewards.py                             # Reward functions
    └── terminations.py                        # Termination conditions
```

## Customization

### Modify Box Properties

Edit `tasks/common_scene/base_scene_pickplace_box_healthcare.py`:

```python
object = RigidObjectCfg(
    spawn=sim_utils.CuboidCfg(
        size=(0.08, 0.08, 0.08),  # Change box size
        mass_props=sim_utils.MassPropertiesCfg(mass=0.3),  # Change mass
        # ... other properties
    ),
)
```

### Adjust Reward Weights

Edit `pick_box_healthcare_g1_29dof_dex3_env_cfg.py`:

```python
@configclass
class RewardsCfg:
    picking_reward = RewTerm(
        func=mdp.compute_box_picking_reward,
        weight=2.0  # Increase reward weight
    )
```

### Change Robot Position

Edit the scene configuration in `pick_box_healthcare_g1_29dof_dex3_env_cfg.py`:

```python
robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex3_wholebody(
    init_pos=(1.0, -0.5, 0.0),  # New position
    init_rot=(1, 0, 0, 0)
)
```

## DDS Communication

The task includes optional DDS (Data Distribution Service) communication for:
- Real-time reward monitoring
- Robot state publishing (50 Hz max)
- Teleoperation integration

DDS is automatically initialized if the DDS module is available. No additional configuration needed.

## Notes

- Make sure `PROJECT_ROOT` environment variable is set before running
- Omniverse Nucleus server must be accessible for loading the healthcare scene
- GPU with CUDA support recommended for real-time simulation
- Contact sensors track 10 frames of history for stable force feedback

## Troubleshooting

**Issue**: Scene not loading
- **Solution**: Verify Omniverse connection and access to `isaac-dev.ov.nvidia.com`

**Issue**: DDS warnings
- **Solution**: DDS is optional. The task works without it. Ignore warnings if not using teleoperation.

**Issue**: Low frame rate
- **Solution**: Reduce `num_envs` or simplify camera observations

## License

Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.  
License: Apache License, Version 2.0

