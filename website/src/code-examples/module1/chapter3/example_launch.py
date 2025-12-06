# website/src/code-examples/module1/chapter3/example_launch.py
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
