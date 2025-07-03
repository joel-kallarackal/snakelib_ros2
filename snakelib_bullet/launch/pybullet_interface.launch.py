from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('snakelib_bullet'),
            'param',
            'sim_params.yaml'
        ])
    )

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('snakelib_description'),
            'rsnake',
            'rsnake.urdf'
        ])
    )

    # Node definition
    pybullet_node = Node(
        package='snakelib_bullet',
        executable='pybullet_interface',  # This must be installed as a script entry point or executable
        name='pybullet_interface',
        output='screen',
        parameters=[LaunchConfiguration('config_file'),
                    {'urdf_path': LaunchConfiguration('urdf_path')}]
    )

    return LaunchDescription([
        config_file_arg,
        urdf_path_arg,
        pybullet_node
    ])
