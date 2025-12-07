---
sidebar_position: 4
title: Simulating Sensors
---

# Simulating Sensors: Eyes and Ears for the Twin

## 1. Sensor Simulation in Gazebo

Gazebo uses **plugins** to simulate sensors. These plugins attach to links in the URDF/SDF and publish data to ROS 2 topics.

### Common Sensors & Plugins

| Sensor Type | Plugin Library | ROS Interface |
| :--- | :--- | :--- |
| **Camera** | `libgazebo_ros_camera.so` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` |
| **LiDAR** | `libgazebo_ros_ray_sensor.so` | `sensor_msgs/LaserScan`, `sensor_msgs/PointCloud2` |
| **IMU** | `libgazebo_ros_imu_sensor.so` | `sensor_msgs/Imu` |
| **GPS** | `libgazebo_ros_gps_sensor.so` | `sensor_msgs/NavSatFix` |

### Configuration Example (SDF)
To add a camera to a robot in Gazebo:

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>my_robot/camera</cameraName>
    <imageTopicName>image_raw</imageTopicName>
    <cameraInfoTopicName>camera_info</cameraInfoTopicName>
    <frameName>camera_link</frameName>
  </plugin>
</sensor>
```

## 2. Sensor Simulation in Unity

Unity uses **GameObjects** combined with C# scripts to simulate sensors. The `ROS-TCP-Connector` is then used to serialize this data and send it to ROS.

### 1. Camera
*   **Mechanism:** Use a standard Unity `Camera` component.
*   **Data Extraction:** Render the camera view to a `RenderTexture`, then read pixels into a `Texture2D`.
*   **ROS Publication:** Encode as JPEG/PNG or raw bytes and publish as `sensor_msgs/CompressedImage` or `sensor_msgs/Image`.

### 2. LiDAR (Raycast)
*   **Mechanism:** Perform physics `Raycast` in a loop (e.g., 360 degrees).
*   **Data Extraction:** Measure hit distances.
*   **ROS Publication:** Populate a `sensor_msgs/LaserScan` array.
*   **Performance:** CPU-based raycasting can be slow. GPU-based compute shaders are preferred for high-resolution point clouds.

### 3. Depth Camera
*   **Mechanism:** Use a Shader that renders scene depth (distance from camera) instead of color.
*   **ROS Publication:** `sensor_msgs/Image` with `32FC1` encoding (32-bit float, 1 channel).
