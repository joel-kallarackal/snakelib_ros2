#!/usr/bin/env python3

import os

import numpy as np
import yaml
from sensor_msgs.msg import Joy
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from snakelib_msgs.msg import SnakeCommand
from rclpy.parameter import Parameter
import yaml
from ament_index_python.packages import get_package_share_directory
import time

import yaml
import os
from ament_index_python.packages import get_package_share_directory

class JoystickTeleop(Node):
    def __init__(self):
        """Initializes the JoystickTeleop

        Args:
        snake_type: string denote the type of snake, e.g. "REU"
        """
        super().__init__('joystick_teleop')
        launch_params_path = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'launch_params.yaml')
        
        with open(launch_params_path, "r") as file:
            launch_params = yaml.safe_load(file)
            self.snake_type = launch_params.get("snake_type")
            self.joystick_name = launch_params.get("joystick_name")
        
        self._param_folder_path = os.path.join(get_package_share_directory('snakelib_control'), 'param/')

        self._loop_rate = 50
        self._snake_command = SnakeCommand()
        self._last_sent = "hold_position"
        self.joy_state = []
        self._mapped_command = ""
        self._x_state = self._y_state = self._z_state = 0
        self._roll = self._pitch = self._yaw = 0
        self._slope_sidewind = 0
        # Direction of general gait
        self._direction = 0
        # Pole curve direction set before pole climb
        self._pole_direction = 0
        # wt direction used for inside or outside pole climbing
        self._wt_dir = 1
        # switch flag used for mode switching and enabling behaviours as the third index in the joystick mapping yaml file
        self._switch_flag = 0
        # flag indicating if the direction for pole climbing is set
        self._has_pole_direction = False

        # It is assumed that the length of axes array is always 6
        # Set this to the number of axes inputs on the joystick
        self._num_axes = 6

        # Default mode
        self.current_mode = "mode__normal"

        # For memory purposes, keep track of the last command sent
        self._prev_mode = ""
        self._prev_index = np.array([])  # Previous indices that were executed in the joy state message
        self._last_index = None  # Index of the last executed index in the joy state message

        # Get snake parameters from parameter server based on snake type
        # Load YAML file
        params_path = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'snake_params.yaml')

        with open(params_path, "r") as file:
            data = yaml.safe_load(file)
        
        self._snake_params = data.get("command_manager").get("ros__parameters").get(f"{self.snake_type}", {})
        self._gait_params = self._snake_params.get("gait_params", {})
        self._tightness = 0

        # Get Joystick mappings
        with open(self._param_folder_path + self.joystick_name + "_mappings.yaml", "r") as ymlfile:
            self._joystick_mappings = yaml.safe_load(ymlfile)

        # Get Joystick to gait mappings
        with open(self._param_folder_path + "joy_to_gait.yaml", "r") as ymlfile:
            self._joy_to_gait = yaml.safe_load(ymlfile)

        # Get speed parameters
        self._speed_step = self._joy_to_gait.get("speed_step", 0.01)
        self._speed_min = self._joy_to_gait.get("speed_min", 0.01)
        self._speed_max = self._joy_to_gait.get("speed_max", 2.0)

        # Get Pole climbing tightness parameters
        self._tightness_step = self._joy_to_gait.get("tightness_step", 0.005)
        self._tightness_min = self._joy_to_gait.get("tightness_min", 0)
        self._tightness_max = self._gait_params.get("pole_climb", {}).get("A_max", 1.5)

        # Get Conical sidewinding parameters
        self._slope_max = self._gait_params.get("conical_sidewinding", {}).get("max_slope", 0.1)
        self._slope_default = self._joy_to_gait.get("slope_default", 0.0)
        self._slope_step = self._joy_to_gait.get("slope_step", 0.005)

        # Initialize subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_cb,
            10)
        self.joy_sub  # prevent unused variable warning
        
        # Initialize publisher
        self.snake_command_pub = self.create_publisher(SnakeCommand, '/snake/command', 1)

        self._mode_logged = False

        timer_period = 1.0 / self._loop_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self._last_warn_time = None

    def timer_callback(self):
        speed_multiplier = 1.0

        ## Command processing logic ##
        if self._mapped_command == "":
            # If the mapped command is nothing, set the command to hold position
            self.command = "hold_position"
            speed_multiplier = 0.0

            # Reset sidewind slope
            self._slope_sidewind = self._slope_default

        else:
            # Otherwise process the input
            self._command_list = self._mapped_command.split("__")

            if self._command_list[0] == "mode":

                # For change to head look mode, cache the previous mode
                if self._command_list[1] == "head_look":
                    self._prev_mode = self.current_mode
                    self.current_mode = "mode__head_look"

                    # Inform the user about the previous mode
                    self.get_logger().info(f"Previous mode: {self._prev_mode.split('__')[1]}")

                # For other mode changes
                else:
                    self._prev_mode = self.current_mode
                    self.current_mode = self._command_list[0] + "__" + str(self._command_list[1])
                
                if self._command_list[1] == "pole_climb":
                    self.command = "home"
                else:
                    self.command = "hold_position"
                    speed_multiplier = 0.0

                # Inform the user about the mode change
                self.get_logger().info(f"Current mode: {self.current_mode.split('__')[1]}")

                # Reset the mapped command to avoid multiple mode changes
                self._mapped_command = ""

            elif self._command_list[0] == "head_look_exit":
                self.current_mode = self._prev_mode

                self.loginfo_throttle(5, f"Current mode: {self.current_mode.split('__')[1]}")

                self.command = self._command_list[0]
                speed_multiplier = 0.0

            elif self._command_list[0] == "home":
                # Home position as defined in snakelib_control/param/snake_params.yaml
                self.command = self._command_list[0]
                speed_multiplier = 0.0
                self.speed_factor = 1
                self._has_pole_direction = False
                self._switch_flag = 0
                self._tightness = self._tightness_min
                self._pole_direction = 0
                self._slope_sidewind = self._slope_default

                # Reset the mode to normal mode
                self.current_mode = "mode__normal"

                # Inform the user about the mode change and home position only every 5 sec to avoid spamming the console
                self.loginfo_throttle(5, "Reset to Home position")
                self.loginfo_throttle(5, f"Current mode: {self.current_mode.split('__')[1]}")

            elif self._command_list[0] == "g":
                # Set selected gait
                self.command = self._command_list[1]
                speed_multiplier = 1.0

                # Add the axes states only for head look gait
                if self.command == "head_look":
                    # TODO: Remove the hardcoded values
                    self._x_state, self._y_state = self.joy_state[2:4]

                if self.command == "head_look_ik":
                    # TODO: Remove the hardcoded values
                    (
                        self._x_state,
                        self._y_state,
                        self._yaw,
                        self._pitch,
                        self._z_state,
                        self._roll,
                    ) = self.joy_state[:6]
                    
                # Reset sidewind slope value if the selected gait is not conical_sidewinding
                if self.command != "conical_sidewinding":
                    self._slope_sidewind = self._slope_default

                # Set _direction based on the joystick input for gaits mapped to buttons
                if self._command_list[2] == "plus":
                    self._direction = 1
                elif self._command_list[2] == "minus":
                    self._direction = -1

            elif self._command_list[0] == "g_pole" and self._has_pole_direction:
                # Only set pole gait if pole direction is set
                self.command = self._command_list[1]
                speed_multiplier = 1.0

            elif self._command_list[0] == "speed" and self.command != "hold_position" and self.command != "home":
                # Increase/decrease speed
                self.speed_factor += self._speed_step if self._command_list[1] == "plus" else -self._speed_step

                self.speed_factor = np.clip(self.speed_factor, self._speed_min, self._speed_max)

            elif self._command_list[0] == "tightness" and self._has_pole_direction:
                # TODO: Add ability to just change tightness and not roll
                # Increase/decrease tightness
                self._tightness += self._tightness_step if self._command_list[1] == "plus" else -self._tightness_step

                self._tightness = np.clip(self._tightness, self._tightness_min, self._tightness_max)

                self.command = "rolling_helix"
                if self._last_sent != "rolling_helix":
                    self._direction = 0

            elif self._command_list[0] == "slope" and self.command == "conical_sidewinding":
                # Increase/decrease conical sidewinding slope
                self._slope_sidewind += self._slope_step if self._command_list[1] == "plus" else -self._slope_step

                self._slope_sidewind = np.clip(self._slope_sidewind, -self._slope_max, self._slope_max)

            elif self._command_list[0] == "pole_direction":
                # Set command to pole direction
                self.command = self._command_list[0]

                # self._direction will be -1 for right curve and +1 for left curve
                self._pole_direction = self._direction

                # Reest the speed factor when starting pole climbing
                self.speed_factor = 1.0

                # Set the flag true to start pole climbing
                self._has_pole_direction = True

            elif self._command_list[0] == "light_toggle":
                # Toggle light in snake head
                # self.declare_parameter('/led', False)
                self.declare_parameter('/led', Parameter.Type.BOOL)
                current_led = self.get_parameter("/led")
                self.set_parameters([Parameter('led', Parameter.Type.BOOL, not current_led)])

            else:
                # Pass case for any other edge cases
                self.command = "hold_position"
                speed_multiplier = 0.0

        ## Publish the command ##
        # Calculate speed_multiplier
        speed_multiplier *= self._direction * self.speed_factor

        # Populate the SnakeCommand message
        self._snake_command.command_name = self.command
        self._snake_command.param_name = ["speed_multiplier"]
        self._snake_command.param_value = [speed_multiplier]

        # Add tightness and wt parameters only when pole climbing direction is set
        if self._has_pole_direction:
            self._snake_command.param_name.extend(["tightness", "pole_direction", "wt_direction"])
            self._snake_command.param_value.extend([self._tightness, self._pole_direction, self._wt_dir])

        # Add x & y state only when head look gait is selected
        if self.command == "head_look":
            self._snake_command.param_name.extend(["x_state", "y_state"])
            self._snake_command.param_value.extend([self._x_state, self._y_state])

        # Add x, y, z, pitch & yaw states only when inverse kinematics based head look gait is selected
        if self.command == "head_look_ik":
            self._snake_command.param_name.extend(["x_state", "y_state", "z_state"])
            self._snake_command.param_value.extend([self._x_state, self._y_state, self._z_state])
            self._snake_command.param_name.extend(["roll", "pitch", "yaw"])
            self._snake_command.param_value.extend([self._roll, self._pitch, self._yaw])

        # Add slope parameter only when conical_sidewinding gait is selected
        if self.command == "conical_sidewinding":
            self._snake_command.param_name.extend(["slope"])
            self._snake_command.param_value.extend([self._slope_sidewind])

        # Publish the SnakeCommand
        self.snake_command_pub.publish(self._snake_command)
        self._last_sent = self._snake_command.command_name



    def joy_cb(self, msg):
        """Callback function for Joy messages

        Extracts any non-zero joystick inputs and sets the corresponding input

        Args:
            msg: Joy message received by subscriber
        """

        # Concatenate the axes and buttons state
        self.joy_state = list(msg.axes) + list(msg.buttons)

        # Extract non-zero axes and buttons states
        index_array = np.nonzero(self.joy_state)[0]

        for i in range(len(index_array)):
            # Set switch flag to 1 if the 'back' button is pressed for mode switching
            if self._joystick_mappings.get(index_array[i]) == "back":
                self._switch_flag = 1

                # Remove the 'back' button from the index array as there is no execution
                index_array = np.delete(index_array, i)
                break

            # Set switch flag to 2 if an enable button with mapping "enable__..." is pressed for enabling a certain behaviour
            elif (
                self._joy_to_gait.get(self.current_mode, {})
                .get(self._joystick_mappings.get(index_array[i]), {})
                .get(self._switch_flag, "")
                .split("__")[0]
                == "enable"
            ):

                # Flip wt when inside pole climbing mode is set
                if (
                    self._joy_to_gait.get(self.current_mode, {})
                    .get(self._joystick_mappings.get(index_array[i]), {})
                    .get(self._switch_flag, "")
                    .split("__")[2]
                    == "in"
                ):

                    self._wt_dir = -1
                else:
                    self._wt_dir = 1

                self._switch_flag = 2

                # Remove the pressed button correseponding to enable behaviour from the index array as there is no execution
                index_array = np.delete(index_array, i)
                break

            else:
                self._switch_flag = 0

        if index_array.size > 0:
            self.peek(index_array)
        else:
            # If no joystick input is pressed, set the command to nothing
            self._mapped_command = ""
            self._prev_index = np.array([])
            self._last_index = None
            self._switch_flag = 0

    def peek(self, index_array):
        """Unpacks current mapped command based on joystick input using the dictionary that
           was initialized from the yaml mapping file

        Args:
            index_array: Array of indices of the non-zero joystick input
        """

        self._prev_index = np.intersect1d(index_array, self._prev_index)

        i = 0
        while i < len(index_array):
            command_name = (
                self._joy_to_gait.get(self.current_mode, {}).get(self._joystick_mappings.get(index_array[i]), {}).get(self._switch_flag, "")
            )

            # Remove all indices that are mapped to nothing
            if command_name == "":
                index_array = np.delete(index_array, i)

            # If index mapped to a command and not the previous executed index, change the mapped command
            elif index_array[i] not in self._prev_index:
                self._mapped_command = command_name

                # Direction set here for gaits that are mapped to axes on joystick
                if index_array[i] < self._num_axes:
                    self._direction = 1 if self.joy_state[index_array[i]] > 0 else -1

                self._prev_index = np.append(self._prev_index, index_array[i])
                self._last_index = index_array[i]
                break

            # Else increment i to check the next index
            else:
                i += 1

        # If all indices are mapped to nothing, set the command to nothing
        if len(index_array) == 0:
            self._mapped_command = ""
            self._prev_index = np.array([])
            self._last_index = None

        # If the last executed index is unpressed then set the mapped command to lowest index in the index_array
        # This prioritizing of lowest index command works for most of the use cases
        elif self._last_index not in self._prev_index:
            self._mapped_command = (
                self._joy_to_gait.get(self.current_mode, {}).get(self._joystick_mappings.get(index_array[0]), {}).get(self._switch_flag, "")
            )

            # Direction set here for gaits that are mapped to axes on joystick
            if index_array[0] < self._num_axes:
                self._direction = 1 if self.joy_state[index_array[0]] > 0 else -1

            self._prev_index = np.array([index_array[0]])
            self._last_index = index_array[0]
    
    def loginfo_throttle(self, period_sec: float, msg: str, key='default'):
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
            self.get_logger().info(msg)
            self._last_warn_time = now

    def run(self):
        """Runs the JoystickTeleop Node"""

        # NOTE: Precision sidewinding - slower sidewind - check on U-snake and implement
        # NOTE: Amplitude manipulation not implemented for now - can add later if needed

        rate = self.create_rate(self._loop_rate)

        # Inform the user about the initial mode
        if not self._mode_logged:
            self.get_logger().info(f"Current mode: {self.current_mode.split('__')[1]}")
            self._mode_logged = True

        self.speed_factor = 1
        self.command = ""

        while rclpy.ok():
            rclpy.spin(self)



def main():
    rclpy.init()
    joystick_teleop = JoystickTeleop()
    try:
        joystick_teleop.run()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        joystick_teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()