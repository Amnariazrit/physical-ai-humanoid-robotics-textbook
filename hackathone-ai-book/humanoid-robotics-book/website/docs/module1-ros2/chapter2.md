# ROS 2 Nodes and Topics: Communication in Robotics

In the previous chapter, we introduced the fundamental concepts of ROS 2 architecture, including nodes and topics. This chapter will delve deeper into these two core elements, providing practical examples of how they facilitate communication within a robotic system using Python.

## Understanding Nodes

A ROS 2 node is an executable program that performs a specific task. Nodes are designed to be modular, allowing complex robotic systems to be broken down into smaller, manageable components. This modularity promotes reusability, simplifies development, and enhances system reliability.

For example, a robot might have separate nodes for:
*   Reading sensor data (e.g., from a LiDAR or camera).
*   Processing sensor data (e.g., object detection, localization).
*   Controlling motors.
*   Planning navigation paths.
*   User interface interaction.

Each node is an independent process that communicates with other nodes through ROS 2's various communication mechanisms.

## Understanding Topics

Topics are the most common way for nodes to exchange data in a ROS 2 system. They implement a publish/subscribe communication model, which is asynchronous and one-way.

### Publish/Subscribe Model

*   **Publisher**: A node that sends data to a named topic is called a publisher. It continuously or periodically publishes messages to a specific topic.
*   **Subscriber**: A node that receives data from a named topic is called a subscriber. It listens for messages on a specific topic.

Nodes don't communicate directly with each other. Instead, they publish messages to or subscribe to messages from named topics. The ROS 2 middleware (DDS) handles the underlying routing of messages between publishers and subscribers.

### Message Types

Every topic has an associated message type. A message type defines the structure and data types of the information that can be sent over that topic. This strict typing ensures that publishers and subscribers agree on the format of the data, preventing misinterpretations.

For example, a message type for a robot's current position might include `x`, `y`, and `z` coordinates, along with a timestamp.

## Python Example: Simple Publisher and Subscriber

Let's create a simple ROS 2 example in Python where a "talker" node publishes a string message to a topic, and a "listener" node subscribes to that topic and prints the received message.

### 1. Create a ROS 2 Package

First, ensure you are in your ROS 2 workspace `src` directory (e.g., `~/ros2_ws/src`). If you don't have a workspace, create one:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Now, create a new ROS 2 Python package called `py_pubsub`:

```bash
ros2 pkg create --build-type ament_python py_pubsub
```

### 2. Implement the Publisher Node (talker)

Navigate into the `py_pubsub` package directory:

```bash
cd py_pubsub
mkdir py_pubsub # Create a directory for Python modules within the package
```

Create a new Python file named `publisher_member_function.py` inside the `py_pubsub` directory (`~/ros2_ws/src/py_pubsub/py_pubsub/publisher_member_function.py`):

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Implement the Subscriber Node (listener)

Create a new Python file named `subscriber_member_function.py` inside the same `py_pubsub` directory (`~/ros2_ws/src/py_pubsub/py_pubsub/subscriber_member_function.py`):

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
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

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Update `setup.py`

Edit the `setup.py` file in your `py_pubsub` package (`~/ros2_ws/src/py_pubsub/setup.py`) to declare your executables:

```python
from setuptools import find_packages, setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pubsub_launch.py']), # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # Replace with your name
    maintainer_email='your_email@example.com', # Replace with your email
    description='ROS 2 Python publisher and subscriber example',
    license='Apache-2.0', # Or your chosen license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)
```
**NOTE**: Remember to replace `your_name` and `your_email@example.com` with your actual information.

### 5. Build and Run the Package

Navigate back to your workspace root (`~/ros2_ws/`) and build your package:

```bash
cd ~/ros2_ws/
colcon build --packages-select py_pubsub
```

After building, you need to source your workspace setup files so that ROS 2 can find your new executables:

```bash
. install/setup.bash # On Linux
```

Now, open two separate terminals.

In the first terminal, run the talker node:

```bash
ros2 run py_pubsub talker
```

In the second terminal, run the listener node:

```bash
ros2 run py_pubsub listener
```

You should see the talker node publishing "Hello ROS 2: X" messages, and the listener node receiving and printing them, demonstrating successful node-to-node communication via a topic.

## Conclusion

This chapter demonstrated how to create and implement ROS 2 nodes in Python that communicate using the publish/subscribe model via topics. Understanding this fundamental mechanism is key to building more complex robotic behaviors. The next chapter will explore services and actions, providing synchronous and goal-oriented communication patterns.
