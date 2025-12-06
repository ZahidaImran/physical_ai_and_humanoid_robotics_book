# website/src/code-examples/module2/chapter2/launch_diff_robot.launch.py
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
