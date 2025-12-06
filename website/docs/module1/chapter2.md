# Module 1 - Chapter 2: Setting Up a ROS 2 Environment & Basic Controls

Now that you understand the core concepts of ROS 2, it's time to get your hands dirty! This chapter will guide you through setting up a complete ROS 2 development environment and creating your first package with a publisher and subscriber. We'll end by controlling a simulated robot with your own code.

## 2.1 Choosing a ROS 2 Distribution

ROS 2 releases, known as **distributions**, are a bundled set of ROS 2 packages. Each distribution has a specific name (e.g., Humble, Iron) and a support lifecycle. For this book, we recommend using **ROS 2 Humble Hawksbill**, which is a Long-Term Support (LTS) release, ensuring you have a stable platform for years to come.

## 2.2 Installation and Setup

The primary supported operating system for ROS 2 is **Ubuntu Linux**. While installation on Windows and macOS is possible, we strongly recommend using Ubuntu for the best experience.

### Linux (Ubuntu) Installation
1.  **Follow the Official Guide**: The most up-to-date installation instructions are always on the official ROS 2 documentation website. Please follow the guide for [installing ROS 2 Humble on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
2.  **Source the setup file**: After installation, you need to "source" the setup file in your terminal to make the ROS 2 commands available. You can add this to your `.bashrc` to do it automatically for every new terminal:
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
3.  **Verify Installation**: Test your installation by running a simple example, like `ros2 run demo_nodes_cpp talker`.

## 2.3 Creating a ROS 2 Workspace

A **workspace** is a directory where you organize, build, and install your ROS 2 packages. It's a best practice to create a workspace for your projects.

1.  Create a workspace directory and a `src` folder inside it:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```
2.  Build the workspace. Even an empty workspace can be "built":
    ```bash
    colcon build
    ```
    `colcon` is the standard build tool for ROS 2.

## 2.4 Creating Your First ROS 2 Package

Now, let's create a Python package inside our workspace.

1.  Navigate to the `src` directory:
    ```bash
    cd ~/ros2_ws/src
    ```
2.  Use the `ros2 pkg create` command:
    ```bash
    ros2 pkg create --build-type ament_python --node-name my_first_node my_package
    ```
    This creates a new directory named `my_package` with the essential files for a Python-based ROS 2 package, including `package.xml` (package manifest) and `setup.py` (build configuration).

## 2.5 Writing a Simple Publisher and Subscriber

Let's put the code examples from Chapter 1 into our new package. You'll create two files inside the `my_package/my_package` directory.

*(You would now add the `publisher.py` and `subscriber.py` code from Chapter 1 into the appropriate files within your new package.)*

## 2.6 Building and Running Your Nodes

1.  **Build the package**: From the root of your workspace (`~/ros2_ws`), run `colcon build`.
2.  **Source the new setup file**: After a successful build, your workspace will have its own setup files. Source the new file to make your package's executables available:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
3.  **Run your nodes**: Open two separate terminals. In the first, run the subscriber:
    ```bash
    ros2 run my_package subscriber_node_name # The name you defined in setup.py
    ```
    In the second, run the publisher:
    ```bash
    ros2 run my_package publisher_node_name # The name you defined in setup.py
    ```
    You should see the subscriber receiving messages from the publisher!

## 2.7 Basic Robot Control

A common task in robotics is sending velocity commands. The standard message type for this is `geometry_msgs/msg/Twist`.

Let's control the `turtlesim`, a simple robot simulator that comes with ROS 2. Here is an example of a node that reads keyboard presses and publishes them as `Twist` messages:

```python title="src/code-examples/module1/chapter2/teleop_keyboard.py"
# NOTE: This script uses the 'termios' module and is intended for Linux/macOS.
# It will not work on Windows.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Turtle!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i':(1,0),
    'o':(1,-1),
    'j':(0,1),
    'l':(0,-1),
    'u':(1,1),
    ',':(-1,0),
    '.':(-1,1),
    'm':(-1,-1),
}

speedBindings={
    'q':(1.1,1.1),
    'z':(.9,.9),
    'w':(1.1,1),
    'x':(.9,1),
    'e':(1,1.1),
    'c':(1,.9),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\t\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    speed = .5
    turn = 1.0
    x = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0.0
                th = 0.0
            else:
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
```

1.  **Run turtlesim**: In a new terminal, start the turtlesim simulation:
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
2.  **Run your teleop node**: In another terminal, run your teleop node:
    ```bash
    ros2 run my_package teleop_node
    ```
    You should now be able to control the turtle in the simulation!

## 2.8 Chapter Summary

Congratulations! You've set up a ROS 2 development environment, created a workspace and a package, and wrote your first ROS 2 nodes. You've also seen how to control a simulated robot by publishing messages.

In the next chapter, we will explore more advanced ROS 2 features, including services, actions, and launch files, to build more complex and capable robotic systems.