--- 
sidebar_position: 2
title: Isaac Sim Setup & Basics
---

# Setting Up Isaac Sim

Isaac Sim is built on the NVIDIA Omniverse platform. It requires a powerful GPU (RTX series) to render photorealistic scenes and compute physics.

## 1. Installation

1.  **Download Omniverse Launcher:** [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/).
2.  **Install Nucleus:** This is the local server for asset management.
3.  **Install Isaac Sim:** Navigate to the "Exchange" tab, search for "Isaac Sim," and install.

## 2. Hello World: A Scripted Simulation

While you can use the GUI, "Infrastructure as Code" requires us to script our simulations.

### The Script (`hello_world_isaac.py`)
Located in `code_examples/module-03-isaac/scripts/`, this script initializes the simulator and drops a cube.

```python
from omni.isaac.kit import SimulationApp

# 1. Start the App (Headless=False means we see the window) 
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

def main():
    world = World()
    world.scene.add_default_ground_plane()

    # Add a Red Cube
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/SimpleRobot",
            name="simple_robot",
            position=np.array([0, 0, 1.0]),
            color=np.array([1.0, 0.0, 0.0]),
        )
    )

    world.reset()

    # Simulation Loop
    for i in range(500):
        world.step(render=True)
        
    simulation_app.close()

if __name__ == "__main__":
    main()
```

### Running the Script
Use the Python interpreter bundled with Isaac Sim:

```bash
# Linux
./python.sh path/to/hello_world_isaac.py

# Windows
python.bat path\to\hello_world_isaac.py
```

## 3. Key Concepts
*   **USD (Universal Scene Description):** The file format used for all assets. It is hierarchical and non-destructive.
*   **Prims:** "Primitives," the nodes in the USD stage tree (e.g., `/World`, `/World/Cube`).
*   **PhysX:** The physics engine backend.
