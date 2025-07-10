from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments with defaults
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(get_package_share_directory('snakelib_bullet'), 'param', 'sim_params.yaml')
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=os.path.join(get_package_share_directory('snakelib_description'), 'reu_snake', 'reu_snake.urdf')
    )

    # Use LaunchConfiguration to refer to arguments
    config_file = LaunchConfiguration('config_file')
    urdf_path = LaunchConfiguration('urdf_path')

    # Path to snake_params.yaml for snakelib_control parameters
    snake_params_file = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'snake_params.yaml')

    return LaunchDescription([
        config_file_arg,
        urdf_path_arg,

        # Load snake_params.yaml as parameters to command_manager and gait_script_node nodes
        # In ROS2, you can specify parameters with a YAML file

        # pybullet_interface node
        Node(
            package='snakelib_bullet',
            executable='pybullet_interface.py',
            name='pybullet_interface',
            output='screen',
            parameters=[config_file,  # Load sim_params.yaml as parameters
                        {'urdf_path': urdf_path}]
        ),

        # command_manager node
        Node(
            package='snakelib_control',
            executable='command_manager_node',
            name='command_manager',
            output='screen',
            parameters=[snake_params_file]  # Load snake_params.yaml parameters
        ),

        # gait_script_node
        Node(
            package='snakelib_control',
            executable='gait_script_node',
            name='gait_script_node',
            output='screen',
            parameters=[snake_params_file]  # Load snake_params.yaml parameters
        ),
    ])
