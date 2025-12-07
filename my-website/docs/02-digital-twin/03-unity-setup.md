---
sidebar_position: 3
title: Unity for Robotics
---

# Unity for Robotics: Advanced Visualization

While Gazebo excels at physics, Unity excels at rendering and interaction. We use the **ROS-TCP-Connector** to bridge the two worlds.

## 1. Setting Up the Environment

### Prerequisites
*   Unity Hub & Editor (2021.3+ recommended).
*   **ROS-TCP-Endpoint** package running on your ROS 2 machine.

### Installation
1.  Open Unity -> Window -> Package Manager.
2.  Add package from git URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`.
3.  In the Unity menu bar, go to `Robotics` -> `ROS Settings` and ensure the IP/Port matches your ROS 2 machine (default 10000).

## 2. Hands-On: A Controlled Robot

We will create a simple script to control a Unity object (Cube + Cylinders) and potentially listen to ROS commands.

### The Controller Script (`SimpleRobotController.cs`)

This script (located in `code_examples/module-02-unity/scripts/`) demonstrates how to:
1.  Read User Input (WASD).
2.  Apply velocity to Unity's `ArticulationBody` components (the physics engine for robotics).
3.  (Optional) Subscribe to a ROS topic.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class SimpleRobotController : MonoBehaviour
{
    // ... (See code_examples for full source)

    void OnMessageReceived(TwistMsg msg)
    {
        // Simple Differential Drive Logic
        float linear = (float)msg.linear.x;
        float angular = (float)msg.angular.z;
        
        SetWheelVelocity(leftWheel, linear - angular);
        SetWheelVelocity(rightWheel, linear + angular);
    }
}
```

## 3. Importing URDFs

Manually building robots with Cubes is tedious. Unity provides a **URDF Importer**:
1.  Install the `URDF Importer` package.
2.  Right-click in the Project view -> "Import Robot from URDF".
3.  Select your `.urdf` file. Unity will automatically generate the GameObjects, ArticulationBodies, and Colliders.

## 4. The ROS-Unity Bridge

To make the connection live:

1.  **In ROS 2:** Run the endpoint.
    ```bash
    ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
    ```
2.  **In Unity:** Press Play. You should see "Connected to ROS" in the console.

Now, your Unity simulation is a node in the ROS network!
