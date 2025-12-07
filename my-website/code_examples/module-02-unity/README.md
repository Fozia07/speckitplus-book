# Unity Project Setup Guide

## Prerequisites
1.  **Unity Hub** & **Unity Editor** (2021.3 LTS or newer recommended).
2.  **ROS-TCP-Endpoint** package running on your ROS 2 machine.

## Step-by-Step Setup

### 1. Create a New Project
*   Open Unity Hub -> New Project -> **3D Core**.
*   Name: `RoboticsSimulation`.

### 2. Install ROS-TCP-Connector
*   In Unity, go to `Window` -> `Package Manager`.
*   Click `+` -> `Add package from git URL`.
*   Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

### 3. Setup Scene
*   Create a Plane (Right Click -> 3D Object -> Plane).
*   Reset its Transform to (0,0,0).

### 4. Import Robot
*   **Simple Cube Robot:**
    *   Create a Cube (Body). Add `ArticulationBody` component.
    *   Create two Cylinders (Wheels). Add `ArticulationBody` to each.
    *   Parent Wheels to Body.
    *   Configure Articulation Types (Revolute for wheels).
*   **URDF Import (Advanced):**
    *   Install `URDF Importer` package from Unity Robotics Hub.
    *   Right Click in Assets -> Import Robot from URDF.

### 5. Add Control Script
*   Drag `SimpleRobotController.cs` onto the Body of your robot.
*   Assign the Left Wheel and Right Wheel Articulation Bodies in the Inspector.
*   Press Play and use WASD / Arrow Keys to move.
