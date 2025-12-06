# Module 2 - Chapter 2: Building Robot & Environment Simulations with Gazebo

This chapter will guide you through the process of building and simulating robot models and environments using Gazebo, a powerful 3D robotics simulator. We'll cover the fundamental description formats for robots and worlds, and integrate them with ROS 2 for control and sensor data visualization.

## 2.1 Introduction to Gazebo

**Gazebo** is an open-source 3D robotics simulator that allows you to accurately simulate populations of robots in complex indoor and outdoor environments. It provides a robust physics engine, high-quality graphics, and interfaces for programming and controlling robots.

**Key features of Gazebo include:**

-   **Physics Engine**: Supports ODE, Bullet, DART, and Simbody for realistic rigid body dynamics.
-   **Sensors**: Emulates various robot sensors like cameras, LiDAR, IMU, sonar, and more.
-   **Models**: Supports loading and simulating 3D models of robots and environmental objects.
-   **Plugins**: Extensible architecture allows for custom behaviors and integrations, including seamless interaction with ROS/ROS 2.

Gazebo's strong integration with ROS 2 makes it an indispensable tool for robotics research and development.

## 2.2 Installing and Setting Up Gazebo with ROS 2

To get started, you'll need to install Gazebo and its ROS 2 integration packages. The specific Gazebo version often corresponds to your ROS 2 distribution (e.g., Gazebo Fortress for ROS 2 Humble).

1.  **Follow the Official ROS 2 Gazebo Installation Guide**: The most reliable and up-to-date instructions can be found on the official ROS 2 documentation. Search for "ROS 2 Gazebo installation [your_ROS2_distribution]". For Humble, this would typically involve installing `ros-humble-gazebo-ros-pkgs` and the specific Gazebo simulator (`ros-humble-gazebo-classic` or `ros-humble-gazebo-ros-gz`).

2.  **Verify Installation**:
    ```bash
    gazebo # Should launch the Gazebo GUI
    ros2 launch gazebo_ros gazebo.launch.py # Should launch Gazebo with ROS 2 integration
    ```

## 2.3 Robot Description Format (URDF and SDF)

To simulate a robot, Gazebo needs a description of its physical properties. Two primary XML-based formats are used:

-   **URDF (Unified Robot Description Format)**:
    -   A standard XML file format for describing the kinematic and dynamic properties of a robot.
    -   Primarily used to describe the robot's structure, including `link` elements (rigid bodies) and `joint` elements (connections between links).
    -   Can define visual (how it looks) and collision (how it interacts physically) properties.
    -   Easily visualized in tools like `rviz2`.
    -   *Limitation*: Cannot describe entire environments or multi-robot systems.

-   **SDF (Simulation Description Format)**:
    -   A more general XML format used by Gazebo to describe everything in a simulation: robots, static environments, lights, sensors, and more.
    -   Can describe aspects that URDF cannot, such as dynamic objects that are not part of a robot's kinematic chain.
    -   Gazebo can often convert URDF files into SDF for simulation, but direct SDF creation offers more control over simulation-specific properties.

## 2.4 Creating Your First Robot Model (URDF)

Let's create a simple **differential drive robot** model using URDF. This robot will have a base link and two wheels.

```xml title="src/code-examples/module2/chapter2/diff_robot.urdf"
<?xml version="1.0"?>
<robot name="differential_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="right_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0.0 -0.11 0.0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0.0 0.11 0.0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

3.  **Visualize in `rviz2`**: Use `ros2 launch urdf_tutorial display.launch.py model:=src/my_diff_robot_description/urdf/my_diff_robot.urdf` (after installing `urdf_tutorial`) to see your robot model.

## 2.5 Integrating Your Robot Model into Gazebo

To bring your URDF robot model into Gazebo, you'll typically create a Gazebo world file and a ROS 2 launch file.

1.  **Create a Gazebo World File** (e.g., `my_robot_world.world`):

    ```xml title="src/code-examples/module2/chapter2/empty_world.world"
    <?xml version="1.0"?>
    <sdf version="1.7">
      <world name="empty_world">
        <light name="sun" type="directional">
          <cast_shadows>1</cast_shadows>
          <pose>0 0 10 0 -0 0</pose>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <direction>-0.5 0.1 -0.9</direction>
          <spot>
            <inner_angle>0</inner_angle>
            <outer_angle>0</outer_angle>
            <falloff>0</falloff>
          </spot>
        </light>
        <model name="ground_plane">
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <surface>
                <friction>
                  <ode>
                    <mu>1.0</mu>
                    <mu2>1.0</mu2>
                  </ode>
                </friction>
              </surface>
            </collision>
            <visual name="visual">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <material>
                <ambient>0.8 0.8 0.8 1</ambient>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <specular>0.8 0.8 0.8 1</specular>
              </material>
            </visual>
          </link>
        </model>
        
        <!-- Add your robot here -->
        <!-- <include>
          <uri>model://my_diff_robot_description</uri>
        </include> -->

      </world>
    </sdf>
    ```

2.  **Create a ROS 2 Launch File**:

    ```python title="src/code-examples/module2/chapter2/launch_diff_robot.launch.py"
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.substitutions import LaunchConfiguration, Command
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    import os

    def generate_launch_description():
        # Get the path to your robot's URDF file
        # For simplicity, we assume the URDF is directly in the code-examples directory for now
        robot_description_pkg_path = get_package_share_directory('diff_robot_pkg')
        urdf_file_path = os.path.join(
            robot_description_pkg_path,
            'urdf',
            'diff_robot.urdf'
        )

        # Robot State Publisher node
        # Publishes the robot's state from the URDF to tf2
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}]
        )

        # Gazebo Launch
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': os.path.join(
                robot_description_pkg_path,
                'worlds', # Assuming you'd have a worlds directory in your package
                'empty_world.world' # Your custom world file
            )}.items()
        )

        # Spawn your robot in Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'differential_robot'],
            output='screen'
        )

        return LaunchDescription([
            # Declare arguments
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation (Gazebo) clock if true'),

            robot_state_publisher_node,
            gazebo_launch,
            spawn_entity,
        ])
    ```

## 2.6 Basic Robot Control in Gazebo (via ROS 2)

Once your robot is in Gazebo, you can control it using the `ros2_control` framework.

1.  **`ros2_control`**: This powerful framework provides a standardized interface between your high-level control code and the robot's hardware (or simulated hardware in Gazebo). You'll need to configure it in your robot's URDF/SDF.
2.  **Control via `Twist` commands**: For a differential drive robot, you can publish `geometry_msgs/msg/Twist` messages to a `/cmd_vel` topic, which `ros2_control` translates into wheel velocities in Gazebo.
3.  **Monitoring Robot State**: Use `ros2 topic echo /joint_states` and `ros2 topic echo /odom` to see the robot's joint positions and estimated odometry in Gazebo.

## 2.7 Simulating Sensors

Gazebo allows you to add various sensors to your robot model and simulate their output.

1.  **Add Sensors to URDF/SDF**: Extend your robot description to include tags for sensors like:
    -   `<sensor type="gpu_lidar" name="lidar">`
    -   `<sensor type="camera" name="camera">`
    -   `<sensor type="imu" name="imu_sensor">`
2.  **Configure Sensor Properties**: Define parameters like resolution, update rate, field of view, and noise characteristics within the sensor tags.
3.  **Visualize Sensor Data in `rviz2`**: Once configured, these sensors will publish data on ROS 2 topics, which you can visualize in `rviz2` (e.g., point clouds from LiDAR, image streams from cameras).

## 2.8 Chapter Summary

In this chapter, you've gained hands-on experience with Gazebo, learning how to define robot models using URDF/SDF, integrate them into simulation environments, and control them via ROS 2. You've also explored how to add and simulate various sensors.

In the next chapter, we will transition to Unity, a different kind of simulation tool, to explore its capabilities for advanced robotics simulation and visualization.