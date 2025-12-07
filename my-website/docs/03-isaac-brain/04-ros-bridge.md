---
sidebar_position: 4
title: The ROS 2 Bridge
---

# Connecting Isaac Sim to ROS 2

The **ROS 2 Bridge** extension allows Isaac Sim to communicate with the rest of your ROS network. It serializes USD data into ROS messages and deserializes ROS commands into physics updates.

## 1. Enabling the Bridge

1.  In Isaac Sim, go to `Window` -> `Extensions`.
2.  Search for `omni.isaac.ros2_bridge`.
3.  Enable it.

## 2. OmniGraph: Wiring the Bridge

Isaac Sim uses **OmniGraph** (Visual Scripting) to define the data flow. You don't write C++ code for the bridge; you connect nodes in a graph editor or via Python.

### Essential Nodes
*   **ROS2 Context:** Initializes the DDS domain.
*   **ROS2 Publish Clock:** Publishes simulation time (`/clock`). Essential because simulation time != real time.
*   **ROS2 Publish Transform Tree:** Publishes the TF tree (`/tf`).
*   **ROS2 Camera Helper:** A high-level node that handles publishing RGB, Depth, and CameraInfo.

### Python Configuration Example

```python
import omni.graph.core as og

# Create a graph that publishes an image every simulation tick
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("PubImage", "omni.isaac.ros2_bridge.ROS2PublishImage"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PubImage.inputs:execIn"),
        ],
    },
)
```

## 3. Best Practices
*   **Use `use_sim_time`:** Always set this parameter to `true` on your ROS nodes when connected to Isaac Sim.
*   **Headless Mode:** When running large training jobs, run Isaac Sim headless (no GUI) to save GPU resources for computation.
