---
sidebar_position: 5
title: Lifecycle Nodes & Parameters
---

# Advanced ROS 2: Lifecycle & Configuration

## 1. ROS 2 Lifecycle Nodes (Managed Nodes)

Lifecycle nodes (or Managed Nodes) introduce a state machine to ROS 2 nodes, allowing for deterministic startup and shutdown sequences. This is critical for safety-critical physical AI systems where sensors and actuators must be initialized in a specific order.

### The State Machine
A lifecycle node has four primary states:
1.  **Unconfigured:** The node is instantiated but not initialized.
2.  **Inactive:** The node is configured (resources allocated) but not processing data.
3.  **Active:** The node is fully operational (publishing/subscribing).
4.  **Finalized:** The node is destroyed.

### Transitions
*   `configure()`: Allocates memory, connects to hardware.
*   `activate()`: Enables publishers/timers.
*   `deactivate()`: Disables publishers/timers but keeps configuration.
*   `cleanup()`: Releases resources, returns to Unconfigured.
*   `shutdown()`: Destroys the node.

### Use Case in Physical AI
*   **Robot Startup:** Ensure the motor controller is `Active` only *after* the safety stop system is `Active`.
*   **Error Handling:** If a sensor fails, `deactivate` the navigation node to stop the robot safely.

## 2. ROS 2 Parameters

Parameters allow nodes to be configured at runtime without recompiling code.

### Key Concepts
*   **Key-Value Pairs:** stored inside the node (e.g., `max_velocity: 2.0`).
*   **Types:** Integer, Float, Boolean, String, Array.
*   **Dynamic Reconfiguration:** Parameters can be changed while the node is running (if callbacks are implemented).

### CLI Commands
*   `ros2 param list`: Show all parameters on a node.
*   `ros2 param get <node_name> <param_name>`: Read a value.
*   `ros2 param set <node_name> <param_name> <value>`: Write a value.
*   `ros2 param dump <node_name>`: Save current params to a YAML file.

### Loading Parameters
Parameters are typically loaded from a YAML file at launch:

```yaml
/navigation_node:
  ros__parameters:
    max_speed: 1.5
    use_lidar: true
    waypoint_timeout: 10
```
