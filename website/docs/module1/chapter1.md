# Module 1 - Chapter 1: Introduction to ROS 2 & Core Concepts

Welcome to the exciting world of robotics! This chapter kicks off our journey by introducing you to the Robot Operating System (ROS), the de facto standard for robotics software development. We'll explore why ROS 2 was created and the fundamental concepts that form the backbone of any ROS 2-powered robot.

## 1.1 What is ROS and Why ROS 2?

**ROS (Robot Operating System)** is not a traditional operating system like Windows or Linux. Instead, it's a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

ROS 1, the original version, revolutionized robotics by providing a standardized way for different parts of a robot's software to communicate with each other. However, as robotics evolved, the needs of developers outgrew what ROS 1 could offer. This led to the development of ROS 2.

**Key advantages of ROS 2 over ROS 1 include:**

-   **Real-time Support**: ROS 2 is designed with real-time control loops in mind, making it suitable for time-critical applications.
-   **Multi-Robot Systems**: It offers built-in support for systems with multiple robots that need to communicate and coordinate.
-   **Embedded Systems**: ROS 2 is more adaptable to smaller, embedded systems with limited resources.
-   **Improved Security**: A robust security model allows for authentication and encryption of messages.

## 1.2 The ROS 2 Graph: A System of Nodes

At its core, a ROS 2 system is a network of processes called **nodes**. Think of this network as the robot's nervous system, with each node being a neuron responsible for a specific task. This network of nodes is called the **ROS 2 Graph**.

-   **Nodes**: A node is the fundamental processing unit in ROS 2. It's a process that performs computation. For example, you might have one node for controlling the robot's wheels, another for processing camera data, and a third for planning the robot's path. Here is a basic example of a ROS 2 node in Python:

```python title="src/code-examples/module1/chapter1/simple_node.py"
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Hello from the simple node!')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 1.3 Core ROS 2 Concepts

Nodes communicate with each other using a few key mechanisms:

-   **Topics**: The most common communication method is asynchronous publish-subscribe messaging via **topics**.
    -   **Analogy**: A public announcement board. A node "publishes" a message to a topic (the board), and any other node that has "subscribed" to that topic will receive the message. This is a one-to-many communication model.
    -   **Message Types**: Every topic has a specific message type, defined in a `.msg` file. This ensures that all nodes communicating on a topic are speaking the same language.

    Here is an example of a publisher node:
    ```python title="src/code-examples/module1/chapter1/publisher.py"
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimplePublisher(Node):
        def __init__(self):
            super().__init__('simple_publisher')
            self.publisher_ = self.create_publisher(String, 'chatter', 10)
            self.timer = self.create_timer(0.5, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello World: {self.i}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1

    def main(args=None):
        rclpy.init(args=args)
        node = SimplePublisher()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

    And here is a corresponding subscriber node:
    ```python title="src/code-examples/module1/chapter1/subscriber.py"
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimpleSubscriber(Node):
        def __init__(self):
            super().__init__('simple_subscriber')
            self.subscription = self.create_subscription(
                String,
                'chatter',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning

        def listener_callback(self, msg):
            self.get_logger().info(f'I heard: "{msg.data}"')

    def main(args=None):
        rclpy.init(args=args)
        node = SimpleSubscriber()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

-   **Services**: For synchronous request-response communication, ROS 2 provides **services**.
    -   **Analogy**: A function call to another machine. A client node sends a request to a server node and waits for a response. This is a one-to-one communication model.
    -   **When to use**: Services are ideal for specific, non-continuous tasks like "get the robot's current position" or "save the current map."
    -   **Service Types**: Similar to topics, services have a defined type in a `.srv` file, which specifies the structure of the request and response.

-   **Actions**: For long-running, feedback-enabled tasks, ROS 2 offers **actions**.
    -   **Analogy**: Ordering a pizza. You place an order (the goal), get updates on its status (feedback), and finally receive the pizza (the result).
    -   **When to use**: Actions are perfect for tasks that take time and where you want to monitor progress, such as navigating to a location or executing a complex manipulation task.
    -   **Action Components**: Actions consist of a goal, feedback, and a result, all defined in an `.action` file.

-   **Parameters**: These are node configuration values that can be set and changed at runtime. This allows you to reconfigure a node's behavior without restarting it.

-   **Launch Files**: For complex systems with many nodes, ROS 2 provides **launch files**. These are scripts that can start up multiple nodes and configure them all at once, making it easy to run your entire robot's software with a single command.

## 1.4 The ROS 2 Command-Line Interface (CLI)

ROS 2 comes with a powerful command-line interface (CLI) tool, `ros2`, that allows you to inspect and interact with your running system.

**Common sub-commands include:**

-   `ros2 run`: Execute a single node.
-   `ros2 topic`: Inspect topics, view messages, and publish messages manually.
-   `ros2 service`: List available services and call them from the command line.
-   `ros2 action`: List available actions and send goals to them.
-   `ros2 param`: Inspect and manage node parameters.
-   `ros2 launch`: Execute a launch file.

## 1.5 Chapter Summary

In this chapter, we've covered the fundamental concepts of ROS 2. You now understand what ROS 2 is, why it's important, and the key building blocks of a ROS 2 system: nodes, topics, services, and actions. You've also been introduced to the `ros2` CLI, your primary tool for interacting with a running ROS 2 system.

In the next chapter, we will put these concepts into practice by setting up your own ROS 2 development environment and writing your first ROS 2 nodes.