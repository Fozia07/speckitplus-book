---
sidebar_position: 5
title: Reinforcement Learning
---

# Reinforcement Learning in Isaac Sim

Isaac Sim's killer feature is **GPU-based Parallel Simulation**. Unlike Gazebo, which simulates one robot per CPU core, Isaac Sim can simulate thousands of robots on a single GPU, accelerating training by orders of magnitude.

## 1. The RL Loop

In Reinforcement Learning (RL), an **Agent** learns a **Policy** by interacting with an **Environment**.

1.  **Observation ($O_t$):** The robot sees the world (joint angles, camera pixels).
2.  **Action ($A_t$):** The policy network outputs a command (joint torques).
3.  **Step:** The physics engine simulates the effect of $A_t$.
4.  **Reward ($R_t$):** The environment scores the action (e.g., +1 for moving forward, -1 for falling).

## 2. OmniIsaacGymEnvs

The standard framework for RL in Isaac Sim is `OmniIsaacGymEnvs`. It wraps the simulator in an OpenAI Gym-like interface.

### Task Definition

A task is defined by a Python class inheriting from `RLTask`.

```python
class MyRobotTask(RLTask):
    def get_observations(self):
        # Return tensor of joint positions and velocities
        # shape: (num_envs, num_observations)
        self.obs_buf[...] = ... 
    
    def calculate_metrics(self):
        # Calculate reward based on distance to target
        # shape: (num_envs, 1)
        reward = ...
        self.rew_buf[:] = reward
        
    def pre_physics_step(self, actions):
        # Apply actions (torques) to the robot articulation
        # shape: (num_envs, num_dof)
        self.robot.set_joint_efforts(actions)
```

## 3. Training Workflow

1.  **Design Task:** Define the observations and rewards in Python.
2.  **Train:** Run the training script (usually using PPO - Proximal Policy Optimization).
    ```bash
    python train.py task=Ant
    ```
3.  **Export:** Save the trained neural network as an `.onnx` file.
4.  **Deploy:** Load the `.onnx` file onto the physical robot (or a single inference instance) to run the policy.
