# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# Omniverse Kit-based "Hello World" for Isaac Sim

from omni.isaac.kit import SimulationApp

# Configuration for the Simulation App
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.robots import Robot
import numpy as np

def main():
    # 1. Initialize the World
    world = World()
    world.scene.add_default_ground_plane()

    # 2. Add a Dynamic Cube (representing a simple robot body)
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/SimpleRobot",
            name="simple_robot",
            position=np.array([0, 0, 1.0]),
            scale=np.array([0.5, 0.5, 0.5]),
            color=np.array([1.0, 0.0, 0.0]),
        )
    )

    # 3. Reset the world to apply physics
    world.reset()

    print("Simulation starting. Press Ctrl+C in terminal to stop if running headless.")

    # 4. Simulation Loop
    for i in range(500):
        world.step(render=True)
        if i % 100 == 0:
            position, orientation = cube.get_world_pose()
            print(f"Step {i}: Robot Position: {position}")

    simulation_app.close()

if __name__ == "__main__":
    main()
