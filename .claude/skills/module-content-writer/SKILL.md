---
name: Module Content Writer
description: Writes clear, structured, and practical module content for technical textbooks, focusing on robotics examples.
version: "1.0"
---

## When to Use

Use this skill when you need to generate new module content for a technical textbook, especially for topics requiring step-by-step explanations and practical, robotics-focused examples.

## Step-by-Step Writing Process

1.  **Understand the Topic:** Clearly define the module's core topic and its learning objectives.
2.  **Outline Structure:** Create a logical flow for the content, including an introduction, main concepts (broken into sub-sections), practical examples, and a summary.
3.  **Draft Explanations:** Write clear and concise explanations for each concept, using simple English and avoiding jargon where possible.
4.  **Develop Examples:** Integrate practical, robotics-focused examples to illustrate concepts. Ensure examples are step-by-step and easy to follow.
5.  **Review and Refine:** Check for clarity, accuracy, coherence, and adherence to quality criteria. Remove any unnecessary theory or complex language.

## Quality Criteria

-   **Clear Structure:** Content is organized logically with clear headings and subheadings.
-   **Step-by-step Explanations:** Complex ideas are broken down into easy-to-understand steps.
-   **Simple English:** Language is accessible to a broad technical audience, avoiding excessive academic jargon.
-   **No Unnecessary Theory:** Focus remains on practical application rather than extensive theoretical background.
-   **Practical, Robotics-focused Examples:** Real-world or simulated robotics examples are used effectively to demonstrate concepts.

## Output Format

The output should be in Markdown format, following a clear hierarchical structure.

```markdown
# Module Title

## Introduction
[Brief introduction to the module topic]

## Concept 1: [Sub-topic]
[Explanation of Concept 1]

### Example: [Example Title for Concept 1]
1.  [Step 1]
2.  [Step 2]
    ```python
    # Code snippet if applicable
    ```
3.  [Step 3]

## Concept 2: [Sub-topic]
[Explanation of Concept 2]

### Example: [Example Title for Concept 2]
1.  [Step 1]
2.  [Step 2]
    ```python
    # Code snippet if applicable
    ```
3.  [Step 3]

## Summary
[Concise summary of key takeaways]
```

## Example

### User Prompt:
"Write a module section explaining ROS2 topics and how to publish a simple string message in Python."

### Generated Content (Example):

```markdown
# ROS2 Communication: Topics

## Introduction
ROS2 (Robot Operating System 2) uses a publish/subscribe model for communication between different parts of a robotic system. One of the fundamental communication mechanisms is **Topics**. Topics are named buses over which nodes can exchange messages.

## Understanding ROS2 Topics

Imagine a bulletin board where different robots can post information or read information from others. Each post category (like "robot_status" or "sensor_data") is a topic.

-   **Nodes:** Independent processes that perform computations (e.g., a node for reading lidar data, another for controlling motors).
-   **Topics:** Named channels for data exchange. Nodes don't communicate directly; they publish messages to topics or subscribe to topics to receive messages.
-   **Messages:** Structured data types that are sent over topics. A message could contain a number, a string, sensor readings, or complex robot states.
-   **Publishers:** Nodes that send messages to a topic.
-   **Subscribers:** Nodes that receive messages from a topic.

This decoupled communication allows for flexible and scalable robotic systems, as nodes don't need to know about each other's existence, only the topics they interact with.

### Example: Basic Topic Communication Flow

1.  **Sensor Node:** Publishes `sensor_data` messages to the `/robot/sensor_readings` topic.
2.  **Processor Node:** Subscribes to `/robot/sensor_readings` to get data, processes it, and publishes `processed_data` to `/robot/processed_sensor_data`.
3.  **Actuator Node:** Subscribes to `/robot/processed_sensor_data` to receive commands based on processed data.

## Publishing a Simple String Message in Python (ROS2)

Let's create a simple ROS2 Python node that publishes a "Hello ROS2" string message to a topic.

### Example: Python Publisher Node

Here's how to create a Python script that acts as a publisher:

1.  **Create a New File:**
    First, create a new Python file, e.g., `simple_publisher.py`, inside your ROS2 package `src` directory.

2.  **Add the Code:**
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String # Import the String message type

    class SimplePublisher(Node):
        def __init__(self):
            super().__init__('simple_publisher') # Node name
            # Create a publisher for the 'chatter' topic, with String message type
            # and a queue size of 10
            self.publisher_ = self.create_publisher(String, 'chatter', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0
            self.get_logger().info('Simple Publisher Node Started')

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello ROS2: {self.i}' # Message content
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1

    def main(args=None):
        rclpy.init(args=args) # Initialize ROS2
        simple_publisher = SimplePublisher()
        rclpy.spin(simple_publisher) # Keep node alive until Ctrl+C
        simple_publisher.destroy_node()
        rclpy.shutdown() # Shutdown ROS2

    if __name__ == '__main__':
        main()
    ```

3.  **Explanation:**
    -   `import rclpy` and `from rclpy.node import Node`: These are standard imports for any ROS2 Python node.
    -   `from std_msgs.msg import String`: We import the `String` message type, which is a common ROS2 message for sending text.
    -   `super().__init__('simple_publisher')`: Initializes the node with the name `simple_publisher`.
    -   `self.create_publisher(String, 'chatter', 10)`: This line creates a publisher.
        -   `String`: The type of message it will publish.
        -   `'chatter'`: The name of the topic it will publish to.
        -   `10`: The queue size, which limits the number of messages buffered if subscribers are not ready.
    -   `self.create_timer(timer_period, self.timer_callback)`: Sets up a timer to call `timer_callback` every `0.5` seconds.
    -   `timer_callback()`: This function is called periodically. It creates a `String` message, sets its `data` field, publishes it, and logs the message.
    -   `rclpy.init(args=args)` and `rclpy.spin(simple_publisher)`: Standard ROS2 initialization and spinning to keep the node running.

4.  **Build and Run (within your ROS2 workspace):**
    ```bash
    # From your workspace root, e.g., ~/ros2_ws
    colcon build --packages-select your_ros2_package
    source install/setup.bash
    ros2 run your_ros2_package simple_publisher
    ```

5.  **Verify with a Subscriber:**
    Open another terminal and run the ROS2 topic echo command to see the messages:
    ```bash
    source install/setup.bash
    ros2 topic echo /chatter
    ```
    You should see the "Hello ROS2: X" messages appearing in the terminal.

## Summary

ROS2 Topics provide a flexible publish/subscribe mechanism for nodes to exchange structured messages. Publishers send data to topics, and subscribers receive data from them, enabling decoupled communication essential for complex robotic systems. We demonstrated how to create a simple Python publisher to send string messages.
```
