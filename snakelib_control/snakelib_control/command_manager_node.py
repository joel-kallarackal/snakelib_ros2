import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node

from snakelib_msgs.msg import HebiSensors, SnakeCommand
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.task import Future
import time

from snakelib_control.scripts.gaitlib_controller import GaitlibController
from snakelib_control.scripts.go_to_position_controller import GoToPositionController
from snakelib_control.scripts.watchdog import Watchdog
from tf_transformations import euler_from_quaternion

import yaml
import os
from ament_index_python.packages import get_package_share_directory

class CommandManager(Node):
    """Manages commands and dispatches different controllers for running the robot

    This module listens for "commands" that tell the robot what behavior and what
    parameters to use for that behavior. For example, it will listen for changes on a
    SnakeCommand topic and then update a GaitlibController to change the open-loop gait
    and gait parameters.

    Every "command" has a corresponding "controller" that executes that command. When
    the command changes, this class takes care of switching between these controllers.
    """

    def __init__(self, **kwargs):
        """Initializes the CommandManager"""
        super().__init__("command_manager" ,**kwargs)

        params_path1 = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'snake_params.yaml')
        
        with open(params_path1, "r") as file:
            data = yaml.safe_load(file)

        params_path2 = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'launch_params.yaml')
        
        with open(params_path2, "r") as file:
            self._snake_type = yaml.safe_load(file).get("snake_type")
        
        self._snake_param = data.get("command_manager").get("ros__parameters").get(f"{self._snake_type}", {})
        self._module_names = self._snake_param.get("module_names")

        self._snake_command = SnakeCommand()  # the last received snake command
        self._loop_rate = data.get("command_manager").get("ros__parameters").get("command_manager_frequency")  # Hz, loop rate in Hz for the run() function

        # Rate in Hz at which sensor data must be received before watchdog is timed out
        self._sensors_watchdog_rate = data.get("command_manager").get("ros__parameters").get("sensors_watchdog_rate")

        # Store the home configuration
        self._home_joint_state = JointState()
        self._home_joint_state.name = self._module_names
        self._home_joint_state.position = self._snake_param.get("home")

        # Subscriber that listens for SnakeCommand messages
        self._snake_command_sub = self.create_subscription(
            SnakeCommand,
            "/snake/command",
            self.snake_command_cb,
            100)
        self._snake_command_sub  # prevent unused variable warning

        # Subscribes to the current joint state from hardware or simulation
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/snake/joint_states",
            self.joint_state_cb,
            10)
        self._joint_state_sub  # prevent unused variable warning
        
        self._hebi_sensor_sub = self.create_subscription(
            HebiSensors,
            "/hebi_sensors/data",
            self.hebi_sensor_cb,
            10)
        self._hebi_sensor_sub  # prevent unused variable warning

        self._head_state_sub = self.create_subscription(
            Pose,
            "/snake/head_state",
            self.head_state_cb,
            10)
        self._head_state_sub  # prevent unused variable warning
        
        # Publishes the desired joint state provided by the current running controller
        self._joint_state_pub = self.create_publisher(JointState, 'snake/joint_commands', 100)

        """Sensor watchdog prevents a command from being sent until the current joint
        state is updated, so initialization of _current_joint_state does not matter, but
        for additional safety we set it to the home joint state.
        """
        self._sensors_watchdog = Watchdog(1.0 / self._sensors_watchdog_rate * pow(10, 9))

        self._elapsed_snake_time = 0

        # TODO: Try to move create arc in GaitlibController
        # Pole climb arc parameters
        self._arc_beta = self._snake_param.get("gait_params").get("pole_climb").get("arc_beta")
        self._imu_dir = self._snake_param.get("gait_params").get("pole_climb").get("imu_dir")

        self.joint_states_received = False
        while not self.joint_states_received:
            rclpy.spin_once(self)
            time.sleep(0.1)
        
        num_modules = len(self._module_names)
        self._prev_cmd = self._desired_joint_state = JointState(position=[], velocity=[np.nan] * num_modules, effort=[np.nan] * num_modules)

        self._prev_cmd.name = self._current_joint_state.name
        self._prev_cmd.position = self._current_joint_state.position

        self._desired_joint_state.name = self._current_joint_state.name
        self._desired_joint_state.position = self._current_joint_state.position

        # Default to holding position in home joint state
        self._controller = GoToPositionController(self._snake_type, self._home_joint_state.position, self._current_joint_state)

        timer_period = 1.0 / self._loop_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self._last_warn_time = None

        self.head_acc_x_before_rolling = 0
        self.head_acc_y_before_rolling = -9.6
        self.direction=1

        self.head_state = [0, 0, 0, -np.pi, 0, 0]

    def head_state_cb(self, msg):
        self.head_state[0] = msg.position.x
        self.head_state[1] = msg.position.y
        self.head_state[2] = msg.position.z

        roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.head_state[3] = roll
        self.head_state[4] = pitch
        self.head_state[5] = yaw

    def snake_command_cb(self, msg):
        """Callback function for SnakeCommand messages

        This only updates the SnakeCommand variable stored in the Manager class. It
        does not invoke any further function calls that modify the active controller,
        because this is taken care of inside run().

        Args:
            msg: SnakeCommand message received by the subscriber.
        """
        self._snake_command = msg

    def joint_state_cb(self, msg):
        """Joint state callback function.

        Args:
            msg: JointState message received by the subscriber.
        """

        self._sensors_watchdog.trigger()
        self._current_joint_state = msg
        if self.joint_states_received==False:
            self.get_logger().info(f"Joint State Initalised.")
            self.joint_states_received=True

    def hebi_sensor_cb(self, msg):
        """HEBI sensor callback function.

        Args:
            msg: HebiSensor message received by the subscriber.
        """

        self._hebi_sensor_msg = msg

    def _get_head_acc(self):
        """Fetches latest acceleration readings form HebiSensor msg.

        Returns:
            [x, y]: accelerations for the head module.
        """

        if not hasattr(self, "_hebi_sensor_msg"):
            return [0, -9.8]
        acc = self._hebi_sensor_msg.lin_acc

        return [acc.x[0], acc.y[0]]

    # TODO: Probably move this to GaitlibController
    def _create_arc(self, pole_direction, beta, imu_dir):
        """Joint angles for creating an arc in given direction.
        Args:
            pole_direction: Direction of the arc
            beta: Beta offset for arc
            imu_dir: IMU direction flip based on snake_type"""

        # Find head start orientation
        head_acc_x, head_acc_y = self._get_head_acc()

        # Choose joints that have most of their motion along ground.
        module_offset = imu_dir if abs(head_acc_x) > abs(head_acc_y) else (not imu_dir)

        inverted_module = -1
        if abs(head_acc_x) > abs(head_acc_y):
            if head_acc_x > 0:
                inverted_module *= -1
        else:
            if head_acc_y > 0:
                inverted_module *= -1

        # Create arc in the given direction.
        arc_angles = np.zeros(len(self._module_names))
        # Account for flipping in hardware
        module_flip = np.ones(len(arc_angles[module_offset::2]))
        module_flip[1::2] *= -1
        arc_angles[module_offset::2] = inverted_module * pole_direction * beta * module_flip

        self._pole_climb_start_angles = arc_angles*2
        self._pole_direction_initialized = True
    

    def start_head_look(self):
        """Runs initialization for starting head look.
        Head look handling
            If starting head_look:
                - Save current joint angles
                - Only move `n` modules closest to the head module
            If exiting head_look:
                - Send saved joint angles
        """

        self._joint_angles_before_head_look = self._prev_cmd.position
        self._head_acc = self._get_head_acc()
        self._started_head_look = True

        # Don't update accelerations when moving the head to avoid
        # singularities.
        self._exiting_head_look = False

    def _kill_head_look(self):
        """Clear head look specific attributes (flags)"""

        if hasattr(self, "_joint_angles_before_head_look"):
            delattr(self, "_joint_angles_before_head_look")
            delattr(self, "_started_head_look")
            delattr(self, "_exiting_head_look")
            self._exiting_head_look = False

    def _kill_pole_direction(self):
        """Clear pole climb specific attributes (flags)"""

        if self.pole_climb_initialized:
            delattr(self, "_pole_direction_initialized")

    def manage_last_command(self):
        """Read the last received command and update controller based on that command."""
        self._cmd_name = cmd_name = self._snake_command.command_name

        if cmd_name == "head_look" or cmd_name == "head_look_ik":
            if self._just_started_head_look:
                self.start_head_look()
            # self._n_exiting_modules = 6 if "ik" in cmd_name else 2
            self._n_exiting_modules = 2 if "ik" in cmd_name else 2 # Changed for HEAD LOOK IK

        if cmd_name != "pole_direction":
            self._kill_pole_direction()

        # Hold position
        if cmd_name in ["head_look_exit", "home", "pole_direction"]:
            if cmd_name == "pole_direction" and not self.pole_climb_initialized:
                self.direction = self._snake_command.param_value[2]
                head_acc = self._get_head_acc()
                self.head_acc_x_before_rolling = head_acc[0]
                self.head_acc_y_before_rolling = head_acc[1]
                self._create_arc(self._snake_command.param_value[2], self._arc_beta, self._imu_dir)

            if cmd_name == "head_look_exit":
                if hasattr(self, "_started_head_look"):
                    self._exiting_head_look = True
                    self.ik_headlook_started = False

            # Set hold (fixed) position depending on the command
            if cmd_name == "home":
                target_position = self._home_joint_state.position
                self._kill_head_look()
            elif (not self._just_started_head_look) and self._exiting_head_look:
                target_position = self._joint_angles_before_head_look
            elif cmd_name == "pole_direction":
                target_position = self._pole_climb_start_angles
            else:
                target_position = self._prev_cmd.position

            if not isinstance(self._controller, GoToPositionController):
                if isinstance(self._controller, GaitlibController):
                    self._elapsed_snake_time = self._controller.snake_time
                self._controller = GoToPositionController(
                    self._snake_type,
                    target_position,
                    self._current_joint_state,
                )
            else:
                self._controller.set_go_to_joint_angles(target_position, self._prev_cmd)

        # Process the last received gaitlib command
        elif GaitlibController.is_gaitlib_command(self._snake_command):

            transition_override = False
            if (not self._just_started_head_look) and self._exiting_head_look:
                transition_override = True
                self._kill_head_look()

            # Check if a new Gaitlib controller needs to be created
            if not isinstance(self._controller, GaitlibController):
                self._controller = GaitlibController(
                    self._snake_type,
                    self._current_joint_state,
                    self._elapsed_snake_time
                )

            if cmd_name == "head_look" or cmd_name == "head_look_ik": # Changed for HEAD LOOK IK
                # Pass head accelerations as a gait param.
                self._snake_command.param_name = list(self._snake_command.param_name)
                self._snake_command.param_value = list(self._snake_command.param_value)
                self._snake_command.param_name.append("head_acc_x")
                self._snake_command.param_value.append(self._head_acc[0])
                self._snake_command.param_name.append("head_acc_y")
                self._snake_command.param_value.append(self._head_acc[1])
                self._snake_command.param_name.append("head_x")
                self._snake_command.param_value.append(self.head_state[0])
                self._snake_command.param_name.append("head_y")
                self._snake_command.param_value.append(self.head_state[1])
                self._snake_command.param_name.append("head_z")
                self._snake_command.param_value.append(self.head_state[2])
                self._snake_command.param_name.append("head_roll")
                self._snake_command.param_value.append(self.head_state[3])
                self._snake_command.param_name.append("head_pitch")
                self._snake_command.param_value.append(self.head_state[4])
                self._snake_command.param_name.append("head_yaw")
                self._snake_command.param_value.append(self.head_state[5])
            
            # elif cmd_name=="rolling":
            #     self._snake_command.param_name = list(self._snake_command.param_name)
            #     self._snake_command.param_value = list(self._snake_command.param_value)
            #     self._snake_command.param_name.append("head_acc_x")
            #     self._snake_command.param_value.append(self.head_acc_x_before_rolling)
            #     self._snake_command.param_name.append("head_acc_y")
            #     self._snake_command.param_value.append(self.head_acc_y_before_rolling)
            #     self._snake_command.param_name.append("direction")
            #     self._snake_command.param_value.append(self.direction)

            self._controller.update_gait(self._snake_command, self._prev_cmd, transition_override=transition_override)

        elif cmd_name != "hold_position":
            self.get_logger().warn(f"Invalid SnakeCommand: {self._snake_command}")

    def get_next_cmd(self):
        """Get next joint commands from the controller/hold position."""
        exiting_head_look = self._exiting_head_look if hasattr(self, "_exiting_head_look") else False
        n_exiting_modules = self._n_exiting_modules if hasattr(self, "_n_exiting_modules") else 0
        if self._cmd_name != "hold_position" or exiting_head_look:
            if isinstance(self._controller, GoToPositionController) and exiting_head_look:
                self._desired_joint_state = self._controller.update(
                    self._current_joint_state, exiting_head_look=exiting_head_look, n_exiting_modules=n_exiting_modules
                )
            else:
                self._desired_joint_state = self._controller.update(self._current_joint_state)
        else:
            self._controller._last_time = self._current_joint_state.header.stamp

        return self._desired_joint_state
    
    def timer_callback(self):
        if not self._sensors_watchdog.timed_out():
            self.manage_last_command()
            desired_joint_state = self.get_next_cmd()
            self._joint_state_pub.publish(desired_joint_state)
            self._prev_cmd = desired_joint_state
        else:
            self.logwarn_throttle(0.25, "Sensor watchdog time exceeded. Joint commands not sent.")



    def run(self):
        """Main function that runs the robot."""
        self.get_logger().info("Running CommandManager")
        rclpy.spin(self) # Once spinning starts, timer_callback gets called


    def logwarn_throttle(self, period_sec: float, msg: str):
        """
        Logs a warning message at most once every `period_sec` seconds.

        Args:
            period_sec (float): Minimum seconds between log messages.
            msg (str): Warning message to log.
            key (str): Identifier to track multiple throttled messages separately.
        """
        now = time.time()
        last_time = self._last_warn_time if self._last_warn_time is not None else 0
        if now - last_time >= period_sec:
            self.get_logger().warn(msg)
            self._last_warn_time = now


    @property
    def pole_climb_initialized(self):
        return hasattr(self, "_pole_direction_initialized")

    @property
    def _just_started_head_look(self):
        return not hasattr(self, "_joint_angles_before_head_look")


def main():
    rclpy.init()
    command_manager = CommandManager()
    try:
        command_manager.run()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        command_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()