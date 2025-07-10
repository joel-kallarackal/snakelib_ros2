# File: snake_launch.py

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Set up config file path
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        # Declare launch argument for the YAML file
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                get_package_share_directory('snakelib_control'),
                'param',
                'snake_params.yaml'
            ),
            description='Path to the YAML config file.'
        ),

        # Control Interface Node with parameters
        Node(
            package='snakelib_hebi',
            executable='hebi_control_interface_node',
            name='hebi_control_interface_node',
            output='screen',
            parameters=[config_file]
        ),

        # Sensor Interface Node with parameters
        Node(
            package='snakelib_hebi',
            executable='hebi_sensor_interface_node',
            name='hebi_sensor_interface_node',
            output='screen',
            parameters=[config_file]
        )
    ])
