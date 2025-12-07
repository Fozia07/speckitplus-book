---
sidebar_position: 3
title: Perception & Graphs
---

# Isaac ROS & Compute Graphs

Modern robotics relies on hardware acceleration. **Isaac ROS** provides a suite of high-performance packages (GEMs) that run on the GPU.

## 1. The Component Graph Model

Instead of writing monolithic nodes, Isaac ROS encourages **Composable Nodes**.
*   **Node:** A single function (e.g., Image Rectification).
*   **Container:** A process that holds multiple nodes, allowing them to pass data via zero-copy shared memory (intra-process communication).

## 2. Hands-On: Visual SLAM

Visual Simultaneous Localization and Mapping (VSLAM) computes a robot's position using only cameras and an IMU.

### The Launch File (`isaac_ros_vslam.launch.py`)

This file (in `code_examples/module-03-isaac/configs/`) demonstrates how to configure the VSLAM node.

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# ... (imports)

def generate_launch_description():
    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_rectified_pose': True,
            'enable_imu_fusion': True
        }],
        remappings=[
            ('stereo_camera/left/image', '/camera/left/image_raw'),
            ('visual_slam/imu', '/imu')
        ]
    )

    # The container runs the node efficiently
    container = ComposableNodeContainer(
        name='vslam_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[vslam_node],
        output='screen'
    )
    # ...
```

## 3. Why Hardware Acceleration?
Running VSLAM on a CPU might consume 100% of a core and run at 10Hz. On a GPU (Jetson Orin), it can run at 60Hz+ with minimal CPU load, leaving room for your application logic.
