---
sidebar_position: 4
title: Actions (Goal/Feedback)
---

# ROS 2 Actions: Long-Running Tasks

Actions are a hybrid communication pattern designed for long-running tasks. They combine:
1.  **Goal:** A request to start a task (like a Service).
2.  **Feedback:** Continuous updates on progress (like a Topic).
3.  **Result:** The final outcome (like a Service).

## Concept Overview

*   **Action Server:** Executes the task (e.g., "Navigate to Waypoint"). It can accept a goal, reject it, or cancel it mid-execution.
*   **Action Client:** Sends the goal and listens for feedback.
*   **Action Interface:** Defines Goal, Result, and Feedback types (e.g., `example_interfaces/action/Fibonacci`).

## Hands-On: Fibonacci Action

We will create an action server that computes the Fibonacci sequence. This simulates a long task (we add a `time.sleep` to make it slow).

### The Code (`simple_action_server.py`)

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class SimpleActionServer(Node):

    def __init__(self):
        super().__init__('simple_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Action server ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Simulate a long computation
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1) # Simulating work

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    simple_action_server = SimpleActionServer()
    rclpy.spin(simple_action_server)

if __name__ == '__main__':
    main()
```

### The Code (`simple_action_client.py`)

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class SimpleActionClient(Node):

    def __init__(self):
        super().__init__('simple_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        # Send goal and register callback for feedback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.sequence))

def main(args=None):
    rclpy.init(args=args)
    action_client = SimpleActionClient()
    action_client.send_goal(10) # Calculate first 10 numbers
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Execution

1.  **Run Action Server:**
    ```bash
    ros2 run ros2_fundamentals simple_action_server
    ```
2.  **Run Action Client:**
    ```bash
    ros2 run ros2_fundamentals simple_action_client
    ```
    You will see the feedback printing incrementally (0, 1, 1, 2, 3...) every second until the sequence is complete.
3.  **CLI Interaction:**
    ```bash
    ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}"
    ```
