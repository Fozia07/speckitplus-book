---
sidebar_position: 2
title: Gazebo & URDF/SDF
---

# Building the Digital Twin in Gazebo

Gazebo is the standard physics simulator for ROS 2. In this chapter, we will build a simple robot simulation.

## 1. The Description Formats: URDF vs. SDF

*   **URDF (Unified Robot Description Format):** The standard for *describing* a robot (kinematics, visuals, collisions). It is preferred for ROS 2 tools like `robot_state_publisher` and `rviz2`.
*   **SDF (Simulation Description Format):** The native format for Gazebo. It describes not just the robot, but the *world* (lighting, physics parameters) and supports closed-loop kinematic chains better than URDF.

> **Workflow:** We typically write in URDF (or xacro) and Gazebo converts it to SDF internally, or we define the simulation environment directly in SDF.

## 2. Hands-On: A Simple Robot (SDF)

We have created a basic two-wheeled robot in `code_examples/module-02-simulation/gazebo_world/models/simple_robot/model.sdf`.

### Key Elements of the SDF
*   **`<link name="chassis">`**: The main body of the robot. We use a simple `<box>` geometry.
*   **`<link name="left_wheel">`**: A cylinder representing the wheel.
*   **`<joint type="revolute">`**: Connects the wheel to the chassis, allowing rotation around a specific axis.

### The World File (`simple.world`)
To simulate the robot, we place it in a "World".

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
    <world name="default">
        <include>
            <uri>model://sun</uri>
        </include>
        <include>
            <uri>model://ground_plane</uri>
        </include>
        <include>
            <uri>model://simple_robot</uri>
            <pose>0 0 0.5 0 0 0</pose>
        </include>
    </world>
</sdf>
```

## 3. Running the Simulation

To run this simulation, you need to tell Gazebo where to find your models.

1.  **Set Model Path:**
    ```bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/code_examples/module-02-simulation/gazebo_world/models
    ```
2.  **Launch Gazebo:**
    ```bash
    gazebo --verbose code_examples/module-02-simulation/gazebo_world/simple.world
    ```

You should see a ground plane and your box-robot sitting on it.

## 4. Connecting to ROS 2

To make this "Twin" useful, we need to bridge it to ROS 2. We do this using the `gazebo_ros_pkgs`.

### The Differential Drive Plugin
Add this to your `model.sdf` inside the `<model>` tag to control it via `/cmd_vel`:

```xml
<plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>
  <left_joint>left_wheel_hinge</left_joint>
  <right_joint>right_wheel_hinge</right_joint>
  <wheel_separation>0.4</wheel_separation>
  <wheel_diameter>0.2</wheel_diameter>
  <command_topic>cmd_vel</command_topic>
  <odometry_topic>odom</odometry_topic>
  <publish_odom>true</publish_odom>
</plugin>
```

Now, publishing a `Twist` message to `/cmd_vel` will move the robot in the simulator!
