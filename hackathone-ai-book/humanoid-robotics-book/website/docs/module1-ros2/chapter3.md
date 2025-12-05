# ROS 2 Services and Actions: Synchronous and Asynchronous Calls

In the previous chapter, we explored ROS 2 topics, which provide an asynchronous, one-way communication mechanism ideal for continuous data streams. This chapter introduces two other critical communication patterns: **Services** for synchronous request/reply interactions, and **Actions** for asynchronous, long-running tasks with feedback and preemption capabilities.

## Understanding ROS 2 Services

ROS 2 Services enable nodes to perform synchronous, blocking communication. When a node needs a specific task to be executed and requires a direct response, it can act as a **client** and send a request to another node acting as a **server**. The server processes the request and sends back a single reply.

### Key Characteristics of Services:

*   **Request/Reply Model**: A client sends a request, and a server sends a reply.
*   **Synchronous**: The client typically waits (blocks) until it receives a reply from the server.
*   **One-time Interaction**: Suitable for tasks that can be completed in a single, well-defined interaction.
*   **Service Definition**: Services are defined using `.srv` files, which specify the structure of the request and reply messages.

### Python Example: Simple Service (Add Two Ints)

Let's create a simple service in Python where a server node adds two integers provided by a client node.

First, define the service type. In your `py_pubsub` package (created in the previous chapter, assuming you've built it), create a `srv` directory:

```bash
cd ~/ros2_ws/src/py_pubsub
mkdir srv
```

Inside the `srv` directory, create a file named `AddTwoInts.srv` with the following content:

```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

Now, modify `~/ros2_ws/src/py_pubsub/package.xml` to include the `build_depend` and `exec_depend` for `rosidl_default_generators` and `example_interfaces`:

```xml
<build_depend>rclpy</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>rosidl_default_generators</build_depend> # Add this line
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>rosidl_default_runtime</exec_depend> # Add this line
<member_of_group>rosidl_interface_packages</member_of_group> # Add this line
```

Also, modify `~/ros2_ws/src/py_pubsub/setup.py` to ensure your service definition is processed:

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pubsub_launch.py']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')), # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 Python publisher and subscriber example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'add_two_ints_server = py_pubsub.service_member_function:main', # Add this line
            'add_two_ints_client = py_pubsub.client_member_function:main', # Add this line
        ],
    },
)
```

Now create the server (`service_member_function.py`) and client (`client_member_function.py`) files inside `~/ros2_ws/src/py_pubsub/py_pubsub/`:

`service_member_function.py` (Server):

```python
import rclpy
from rclpy.node import Node

from py_pubsub.srv import AddTwoInts # Import your custom service type


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

`client_member_function.py` (Client):

```python
import sys

import rclpy
from rclpy.node import Node

from py_pubsub.srv import AddTwoInts # Import your custom service type


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (minimal_client.req.a, minimal_client.req.b, response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Build and Run:

```bash
cd ~/ros2_ws/
colcon build --packages-select py_pubsub
. install/setup.bash # Source your workspace
```

Open two terminals. In the first, start the server:

```bash
ros2 run py_pubsub add_two_ints_server
```

In the second, start the client, providing two integers as arguments:

```bash
ros2 run py_pubsub add_two_ints_client 2 3
```

The server should log the incoming request, and the client should log the result `5`.

## Understanding ROS 2 Actions

ROS 2 Actions extend the concept of services for long-running, asynchronous operations that may provide periodic feedback and can be preempted (cancelled). They are ideal for tasks like "move robot to a goal position" or "pick up an object," where the execution takes time and you might want to monitor progress or stop the action midway.

### Key Characteristics of Actions:

*   **Goal/Feedback/Result Model**:
    *   **Goal**: The request sent by the client to initiate the action.
    *   **Feedback**: Periodic updates from the server about the progress of the action.
    *   **Result**: The final outcome of the action (success, failure, aborted).
*   **Asynchronous**: The client doesn't block while waiting for the action to complete.
*   **Preemption**: A client can request to cancel an active goal on the server.
*   **Action Definition**: Actions are defined using `.action` files, specifying the structure of the goal, feedback, and result messages.

### Python Example: Simple Action (Fibonacci)

Let's create a simple action in Python where an action server computes a Fibonacci sequence up to a given order, and an action client requests this computation.

Define the action type. In your `py_pubsub` package `srv` directory, create an `action` directory. Change `py_pubsub` to `action_tutorials_interfaces`

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python action_tutorials_interfaces
cd action_tutorials_interfaces
mkdir action
```

Inside `action_tutorials_interfaces/action`, create `Fibonacci.action` with the content:

```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] sequence
```

Modify `~/ros2_ws/src/action_tutorials_interfaces/package.xml` and `~/ros2_ws/src/action_tutorials_interfaces/setup.py` similar to how you did for services, but for actions.

Now create the server (`fibonacci_action_server.py`) and client (`fibonacci_action_client.py`) files inside `~/ros2_ws/src/py_pubsub/py_pubsub/`:

`fibonacci_action_server.py` (Server):

```python
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci # Correct import path


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            sequence.append(sequence[i] + sequence[i-1])
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.sequence))
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info('Result: {0}'.format(result.sequence))
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)

    fibonacci_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

`fibonacci_action_client.py` (Client):

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci # Correct import path


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

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
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()
    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

Build and Run:

```bash
cd ~/ros2_ws/
colcon build --packages-select py_pubsub action_tutorials_interfaces
. install/setup.bash # Source your workspace
```

Open two terminals. In the first, start the action server:

```bash
ros2 run py_pubsub fibonacci_action_server
```

In the second, start the action client:

```bash
ros2 run py_pubsub fibonacci_action_client
```

You should see the server executing the Fibonacci sequence, sending feedback to the client, and finally a result.

## Conclusion

Services and Actions provide powerful mechanisms for synchronous request/reply and asynchronous long-running task management in ROS 2. By combining them with topics, you can build sophisticated and robust robotic applications that effectively manage various communication needs. The next chapter will explore Python integration in more detail.
