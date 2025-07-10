from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    snake_type = LaunchConfiguration('snake_type')
    num_modules = LaunchConfiguration('num_modules')
    config_file = LaunchConfiguration('config_file')

    # Package paths
    snakelib_control_path = get_package_share_directory('snakelib_control')
    snakelib_bullet_path = get_package_share_directory('snakelib_bullet')
    snakelib_description_path = get_package_share_directory('snakelib_description')
    xacro_path = get_package_share_directory('xacro')  # xacro used as a script

    # Construct URDF path
    xacro_file = os.path.join(snakelib_description_path, f"{snake_type.perform(None)}_snake", f"{snake_type.perform(None)}_snake.xacro")
    urdf_output = os.path.join(snakelib_description_path, f"{snake_type.perform(None)}_snake", f"{snake_type.perform(None)}_instance.urdf")

    # Return launch description
    return LaunchDescription([
        DeclareLaunchArgument('snake_type', default_value='REU'),
        DeclareLaunchArgument('num_modules', default_value='16'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(snakelib_bullet_path, 'param', 'sim_params.yaml')),

        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='snake_joy',
            respawn=True,
            parameters=[{'dev': '/dev/input/js0'}]
        ),

        # Teleop node
        Node(
            package='snakelib_control',
            executable='joystick_teleop_node',
            name='joystick_teleop_node',
            output='screen'
        ),

        # Command manager node
        Node(
            package='snakelib_control',
            executable='command_manager_node',
            name='command_manager_node',
            output='screen',
            parameters=[
                os.path.join(snakelib_control_path, 'param', 'snake_params.yaml'),
                {'snake_type': snake_type}
            ]
        ),

        # PyBullet node
        Node(
            package='snakelib_bullet',
            executable='pybullet_interface.py',
            name='pybullet_interface',
            output='screen',
            parameters=[config_file]
        ),
    ])
