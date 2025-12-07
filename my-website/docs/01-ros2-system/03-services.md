--- 
sidebar_position: 3
title: Services (Request/Response)
---

# ROS 2 Services: Asking for Permission

Services represent a **Synchronous** Request/Response pattern. Unlike topics, services are not continuous streams; they are used for discrete transactions.

## Concept Overview

*   **Server:** A node that offers a functionality (e.g., "Reset Odometry", "Compute IK", "Add Two Ints").
*   **Client:** A node that requests that functionality and waits for the result.
*   **Service Type:** Defines the Request structure and the Response structure (e.g., `example_interfaces/srv/AddTwoInts`).

## Hands-On: Service Server

We will create a service that takes two integers and returns their sum.

### The Code (`simple_service_server.py`)

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleServiceServer(Node):

    def __init__(self):
        super().__init__('simple_service_service')
        # Create a service named 'add_two_ints'
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server ready to add two ints.')

    def add_two_ints_callback(self, request, response):
        # Process request and populate response
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On: Service Client

The client sends a request and waits for the answer.

### The Code (`simple_service_client.py`)

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleServiceClient(Node):

    def __init__(self):
        super().__init__('simple_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Block until the service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # Send async request
        self.future = self.cli.call_async(self.req)
        # Wait for the future to complete
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    simple_service_client = SimpleServiceClient()
    
    a = 6
    b = 7
    response = simple_service_client.send_request(a, b)
    simple_service_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (a, b, response.sum))

    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Execution

1.  **Run Server:**
    ```bash
    ros2 run ros2_fundamentals simple_service_server
    ```
2.  **Run Client (in new terminal):**
    ```bash
    ros2 run ros2_fundamentals simple_service_client
    ```
    *Output:* `Result of add_two_ints: for 6 + 7 = 13`
3.  **Call via CLI:**
    You can also call the service manually:
    ```bash
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
    ```
