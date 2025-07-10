from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='snakelib_gui',
            executable='SnakeLib_GUI',
            name='SnakeLib_GUI',
            output='log'
        )
    ])
