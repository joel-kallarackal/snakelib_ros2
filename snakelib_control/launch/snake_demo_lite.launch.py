import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression

def generate_launch_description():
    # Declare launch arguments
    snake_type_arg = DeclareLaunchArgument(
        'snake_type',
        default_value='REU',
        description='Type of snake'
    )
    num_modules_arg = DeclareLaunchArgument(
        'num_modules',
        default_value='16',
        description='Number of modules'
    )

    # TODO: Load from a param file 
    snake_name = "REU"

    num_modules = LaunchConfiguration('num_modules')

    # Paths
    snakelib_control_dir = get_package_share_directory('snakelib_control')
    snakelib_description_dir = get_package_share_directory('snakelib_description')
    joy_dev = '/dev/input/js1'  # adjust as needed

    # Param files
    snake_params_file = os.path.join(snakelib_control_dir, 'param', 'snake_params.yaml')

    file_path = f"{snake_name}_snake/{snake_name}_snake.xacro"

    xacro_file = PathJoinSubstitution([
        snakelib_description_dir,
        file_path
    ])

    # Run xacro to generate URDF at launch time:
    robot_description_content = Command([
        'xacro ',
        xacro_file,
        ' number_of_modules:=', num_modules
    ])

    return LaunchDescription([
        snake_type_arg,
        num_modules_arg,

        # Parameters that were <param> tags in ROS1
        # snake_type param
        # Node(
        #     package='rclpy', executable='parameter_bridge', output='screen',  # dummy node, optional
        #     parameters=[{'snake_type': snake_type}]
        # ),

        # joystick_name param as parameter to relevant nodes
        # We'll add joystick_name param explicitly to joy node below

        # Load snake_params.yaml parameters into nodes that need them (assumed)
        # You can also declare a standalone node just for parameters if you want

        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='snake_joy',
            respawn=True,
            parameters=[{'dev': joy_dev, 'joystick_name': 'logi_cordless_rumblepad2'}]
        ),

        # Teleop node
        Node(
            package='snakelib_control',
            executable='joystick_teleop_node',
            name='joystick_teleop_node',
            output='log',
            parameters=[snake_params_file]
        ),

        # Command Manager Node
        Node(
            package='snakelib_control',
            executable='command_manager_node',
            name='command_manager',
            output='log',
            parameters=[snake_params_file]
        ),

        # Hebi nodes with respawn=True
        Node(
            package='snakelib_hebi',
            executable='hebi_control_interface_node',
            name='hebi_control_interface_node',
            output='log',
            respawn=True,
            parameters=[snake_params_file]
        ),
        Node(
            package='snakelib_hebi',
            executable='hebi_sensor_interface_node',
            name='hebi_sensor_interface_node',
            output='log',
            respawn=True,
            parameters=[snake_params_file]
        ),

        # robot_description and robot_model parameters
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
    ])
