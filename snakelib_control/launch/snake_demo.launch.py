from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    snake_type_arg = DeclareLaunchArgument(
        'snake_type', default_value='REU', description='Snake type')
    num_modules_arg = DeclareLaunchArgument(
        'num_modules', default_value='16', description='Number of modules')
    basic_gui_arg = DeclareLaunchArgument(
        'basic_gui', default_value='true', description='Launch basic GUI or full GUI')

    snake_type = LaunchConfiguration('snake_type')
    num_modules = LaunchConfiguration('num_modules')
    basic_gui = LaunchConfiguration('basic_gui')

    snakelib_control_dir = get_package_share_directory('snakelib_control')
    snakelib_camera_dir = get_package_share_directory('snakelib_camera')
    snakelib_gui_dir = get_package_share_directory('snakelib_gui')

    snake_demo_lite_launch = os.path.join(snakelib_control_dir, 'launch', 'snake_demo_lite.launch.py')
    camera_launch = os.path.join(snakelib_camera_dir, 'launch', 'camera.launch.py')
    snakelib_gui_launch = os.path.join(snakelib_gui_dir, 'launch', 'snakelib_gui.launch.py')

    return LaunchDescription([
        snake_type_arg,
        num_modules_arg,
        basic_gui_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(snake_demo_lite_launch),
            launch_arguments={
                'snake_type': snake_type,
                'num_modules': num_modules,
                'basic_gui': basic_gui
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(snakelib_gui_launch)
        ),
    ])
