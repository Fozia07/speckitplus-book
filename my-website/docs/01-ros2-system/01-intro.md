---
sidebar_position: 1
title: Introduction & Core Concepts
---

# Introduction to the Robotic Nervous System (ROS 2)

## Module Objectives

This module serves as the foundation for building physical AI systems. By the end of this module, you will be able to:

*   **Understand the ROS 2 Architecture:** Explain the roles of Nodes, Executors, and the Data Distribution Service (DDS) middleware.
*   **Master Communication Patterns:** Differentiate between and implement Topics, Services, and Actions for inter-process communication.
*   **Manage Development Workspaces:** Create, configure, and build ROS 2 packages using `colcon`.
*   **Utilize Introspection Tools:** proficiently use the `ros2` command-line interface (CLI) to debug and monitor running systems.
*   **Design Modular Systems:** Apply best practices for node composition and reusability.

## Key ROS 2 Libraries and Tools

A robust physical AI development environment relies on a suite of standard tools. We will focus on the following:

| Tool/Library | Description |
| :--- | :--- |
| **`rclpy` / `rclcpp`** | The standard client libraries for Python and C++. We will primarily use `rclpy` for high-level logic and `rclcpp` for performance-critical components. |
| **`ros2cli`** | The command-line interface entry point. Commands like `ros2 topic list`, `ros2 node info`, and `ros2 run` are essential. |
| **`colcon`** | The standard build tool for ROS 2. It iterates over packages in a workspace and builds them in dependency order. |
| **`rviz2`** | The primary 3D visualization tool for ROS. Essential for viewing sensor data, robot states, and map information. |
| **`rqt`** | A Qt-based framework for GUI development. We use plugins like `rqt_graph` to visualize the node network topology. |
| **`rosbag2`** | The recording and playback tool. Critical for capturing sensor data sessions for offline analysis and algorithm testing. |
| **`rosdep`** | A command-line tool for installing system dependencies for source code. |
| **`gazebo_ros_pkgs`** | Wrappers and tools that interface ROS 2 with the Gazebo simulation environment (covered in Module 2). |
| **`nav2`** (Navigation 2) | A professional-grade navigation stack for mobile robots. |
| **`moveit2`** | The state-of-the-art motion planning framework for manipulation. |

## ROS 2 Communication Patterns

Understanding *how* data moves between nodes is critical. ROS 2 provides three primary paradigms:

### 1. Topics (Publish / Subscribe)
*   **Mechanism:** Many-to-many, asynchronous communication.
*   **Use Case:** Continuous data streams where the latest data is most important.
*   **Examples:** Lidar scans (`/scan`), camera images (`/camera/image_raw`), robot velocity commands (`/cmd_vel`).

### 2. Services (Request / Response)
*   **Mechanism:** One-to-one, synchronous (blocking) communication.
*   **Use Case:** Short-duration operations that return a result or modify the system state.
*   **Examples:** Resetting a sensor, querying a map location, checking if a motor is enabled.

### 3. Actions (Goal / Feedback / Result)
*   **Mechanism:** One-to-one, asynchronous communication with steady feedback.
*   **Use Case:** Long-running tasks that might need to be preempted or monitored.
*   **Examples:** Navigating to a waypoint, executing a complex arm trajectory, charging a battery.
