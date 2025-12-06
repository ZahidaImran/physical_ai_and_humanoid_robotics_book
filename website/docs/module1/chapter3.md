# Module 1 - Chapter 3: Advanced ROS 2 for Complex Systems

Welcome to the final chapter of Module 1! Having mastered the basics, you're now ready to explore the advanced features of ROS 2 that enable you to build truly robust and scalable robotic systems. This chapter covers parameters, launch files, custom interfaces, and the critical concept of Quality of Service (QoS).

## 3.1 A Deeper Dive into Services and Actions

We introduced Services and Actions in Chapter 1. Let's solidify when to use each:

-   **Use a Service** for quick, transactional request/response interactions. If the task is fast and you need an immediate answer, a service is the right choice. *Example: "What is the robot's current battery level?"*
-   **Use an Action** for long-running, asynchronous tasks where you need to monitor progress and can potentially cancel the goal. *Example: "Navigate to the kitchen," which might take several seconds or minutes and provide feedback along the way.*

## 3.2 ROS 2 Parameters

**Parameters** allow you to configure your nodes externally without changing the code. This is incredibly useful for tuning and adapting your robot's behavior.

-   **Declaring and Using Parameters**: In your Python node, you can declare a parameter and get its value. Here is an example of a node with a configurable parameter:

    ```python title="src/code-examples/module1/chapter3/parameter_node.py"
    import rclpy
    from rclpy.node import Node

    class ParameterNode(Node):
        def __init__(self):
            super().__init__('parameter_node')
            self.declare_parameter('my_parameter', 'world')
            self.timer = self.create_timer(1, self.timer_callback)

        def timer_callback(self):
            my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
            self.get_logger().info(f'Hello {my_param}!')

    def main(args=None):
        rclpy.init(args=args)
        node = ParameterNode()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

-   **Dynamic Reconfiguration**: Parameters can be changed on the fly while a node is running using the `ros2 param` command-line tool.
-   **Loading from a YAML File**: For complex systems, it's common to define all your parameters in a YAML file and load them at startup using a launch file.

## 3.3 Launch Files for Complex Systems

Manually running each node in a complex system is tedious and error-prone. **Launch files** are Python scripts that automate this process.

A ROS 2 launch file allows you to:
-   Launch multiple nodes at once.
-   Pass parameters to your nodes from a YAML file.
-   Remap topic names to connect nodes in flexible ways.
-   Include other launch files to create modular and reusable system descriptions.

Here is an example of a simple launch file that starts our parameter node and the turtlesim simulator:

```python title="src/code-examples/module1/chapter3/example_launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='parameter_node',
            name='my_parameter_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'my_parameter': 'earth'}
            ]
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        )
    ])
```

## 3.4 Structuring a Complex Project

As your projects grow, you'll need to create your own custom message, service, and action types.

-   **Creating Custom Interfaces**: You define these interfaces in `.msg`, `.srv`, and `.action` files within your package. For example, you could create a `HardwareStatus.msg` to report your robot's sensor readings.

    ```
    # src/custom_interfaces/msg/HardwareStatus.msg
    int64 temperature
    bool are_motors_ready
    string debug_message
    ```

-   **The Build Process**: To use these custom interfaces, you need to add specific directives to your `package.xml` and `CMakeLists.txt` (even for Python packages), and then build your workspace with `colcon build`. This process generates the necessary Python code for your new interfaces.
-   **Multi-Package Repositories**: For large systems, it's common practice to organize your code into multiple packages within a single repository (a "monorepo"). For example, you might have one package for robot hardware drivers, another for perception, and a third for navigation.

## 3.5 Quality of Service (QoS)

**Quality of Service (QoS)** is a powerful and unique feature of ROS 2 that allows you to configure how messages are sent and received. It's a set of rules that governs the "quality" of the communication.

Key QoS policies include:
-   **Reliability**: `RELIABLE` guarantees delivery, while `BEST_EFFORT` is faster but may drop messages. Use `RELIABLE` for critical data like commands and `BEST_EFFORT` for high-frequency sensor data where it's okay to miss a few messages.
-   **Durability**: `TRANSIENT_LOCAL` allows new subscribers to receive previously published messages, which is great for topics that publish status or configuration. `VOLATILE` means new subscribers only get messages published after they subscribe.
-   **History**: `KEEP_LAST` stores a limited number of recent messages for subscribers, while `KEEP_ALL` stores all messages (use with caution to avoid memory issues).

Choosing the right QoS profile is crucial for building robust robotic systems.

## 3.6 Chapter Summary

In this chapter, you've learned about the advanced features that make ROS 2 a powerful framework for robotics. You now understand how to use parameters for configuration, launch files for automation, custom interfaces for structured data, and QoS for reliable communication.

You have now completed Module 1 and have a strong foundation in ROS 2. In Module 2, we will take these skills and apply them to the world of simulation, where we will create a digital twin of our robot and learn how to test our code without needing physical hardware.