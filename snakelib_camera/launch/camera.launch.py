from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    camera_config_file = LaunchConfiguration('camera_config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_config_file',
            default_value=os.path.join(
                get_package_share_directory('snakelib_camera'),
                'param',
                'camera_params.yaml'
            ),
            description='Path to camera configuration file'
        ),

        Node(
            package='snakelib_camera',
            executable='toggle_streaming',
            name='toggle_streaming',
            output='screen'
        ),

        Node(
            package='snakelib_camera',
            executable='pinhole_node',
            name='pinhole_node',
            output='screen',
            parameters=[camera_config_file]
        ),

        Node(
            package='snakelib_camera',
            executable='thermal_node',
            name='thermal_node',
            output='screen',
            parameters=[camera_config_file]
        ),

        Node(
            package='snakelib_camera',
            executable='image_processing_node',
            name='image_processing_node',
            output='screen',
            parameters=[camera_config_file]
        ),

        # Uncomment if RViz is needed
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz',
        #     arguments=['-d', os.path.join(
        #         get_package_share_directory('snakelib_camera'),
        #         'rviz',
        #         'rviz_feed.rviz')],
        #     output='screen'
        # )
    ])
