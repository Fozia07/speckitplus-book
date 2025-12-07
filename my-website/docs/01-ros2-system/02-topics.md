---
sidebar_position: 2
title: Topics (Publish/Subscribe)
---

# ROS 2 Topics: The Nervous System's Signals

Topics are the primary way data moves in a ROS 2 system. They function on a **Publish-Subscribe** model, allowing for many-to-many communication.

## Concept Overview

*   **Publisher:** A node that generates data (e.g., a LiDAR sensor, a camera, a path planner).
*   **Subscriber:** A node that consumes data (e.g., a motor controller, a mapping algorithm).
*   **Topic:** The named channel (e.g., `/scan`, `/cmd_vel`) where messages are exchanged.
*   **Message Type:** The data structure definition (e.g., `std_msgs/String`, `sensor_msgs/LaserScan`).

## Hands-On: A Simple Publisher

We will create a node that publishes a "Hello World" message with a counter.

### The Code (`simple_publisher.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        # Create a publisher on topic 'topic' with a queue size of 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components
1.  **`create_publisher`**: Registers the node as a data source for the `topic`.
2.  **`create_timer`**: Sets up a periodic callback (0.5s) to generate data.
3.  **`publish`**: Sends the message to the DDS middleware for distribution.

## Hands-On: A Simple Subscriber

Now we create a node to listen to that topic.

### The Code (`simple_subscriber.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        # Create a subscription to 'topic'
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components
1.  **`create_subscription`**: Registers the node to receive data from `topic`.
2.  **`listener_callback`**: This function is triggered *every time* a new message arrives.

## Execution

1.  **Build:**
    ```bash
    colcon build --packages-select ros2_fundamentals
    ```
2.  **Run Publisher:**
    ```bash
    source install/setup.bash
    ros2 run ros2_fundamentals simple_publisher
    ```
3.  **Run Subscriber (in a new terminal):**
    ```bash
    source install/setup.bash
    ros2 run ros2_fundamentals simple_subscriber
    ```

You should see the publisher printing "Publishing..." and the subscriber printing "I heard...".
