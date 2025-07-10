from functools import partial
import rclpy
import hebi
import numpy as np
from sensor_msgs.msg import JointState
import time
from rclpy.parameter import Parameter

from hebi_interface import HEBIROSWrapper

class HEBIControlROSWrapper(HEBIROSWrapper):
    r"""Driver node that commands targets to the HEBI modules from the `joint_command` topic."""
    def __init__(self):
        # Establish communication with the HEBI modules.
        super().__init__('hebi_control_interface')

        # Start joint command subscriber.
        self.joint_command_topic = "/snake/joint_commands"
        self.joint_command_sub = self.create_subscription(
            JointState,
            self.joint_command_topic,
            self.joint_command_callback,
            10)
        self.joint_command_sub  # prevent unused variable warning

        # Initialize command message buffer.
        self.robot_cmd = hebi.GroupCommand(self.num_modules)

        # With default values, don't use if launching through a launch file
        # self.declare_parameter('snake_type', "REU")
        # self.declare_parameter(self.get_parameter("/snake_type") + "/max_torque", 1.5)
        # self.declare_parameter(self.get_parameter('snake_type').value + "/zero_offset", [0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.declare_parameter('snake_type', Parameter.Type.STRING)
        self.declare_parameter(self.get_parameter("/snake_type") + "/max_torque", Parameter.Type.DOUBLE)
        self.declare_parameter(self.get_parameter('snake_type').value + "/zero_offset", Parameter.Type.DOUBLE_ARRAY)
        
        self._zero_offset = self.get_parameter(self.get_parameter('snake_type').value + "/zero_offset")
        if self._zero_offset == '':
            self._zero_offset = None

        self._last_warn_time = {}

    def logwarn_throttle(self, period_sec: float, msg: str, key='default'):
        """
        Logs a warning message at most once every `period_sec` seconds.

        Args:
            period_sec (float): Minimum seconds between log messages.
            msg (str): Warning message to log.
            key (str): Identifier to track multiple throttled messages separately.
        """
        now = time.time()
        last_time = self._last_warn_time.get(key, 0)
        if now - last_time >= period_sec:
            self.get_logger().warn(msg)
            self._last_warn_time[key] = now

    def send_joint_commands(self):
        r"""Sends joint commands to the module."""
        if np.isnan(self.robot_cmd.position).any() and np.isnan(self.robot_cmd.velocity).any() and np.isnan(self.robot_cmd.effort).any():
            self.logwarn_throttle(100, "No joint command sent.")
            return

        self.robot.send_command(self.robot_cmd)

    def joint_command_callback(self, msg):
        r"""Processes and sends joint commands to the HEBI modules.

        Populates the robot_cmd object with appropriate fields from the received msg after ordering and filtering
        received messages.

        Args:
            msg (JointState): JointState message containing the joint commands.

        """
        self.robot_cmd.position, self.robot_cmd.velocity, self.robot_cmd.effort = map(
            partial(self.sort_joint_commands, names_list=msg.name),
            [msg.position, msg.velocity, msg.effort],
        )

        if self._zero_offset is not None:
            self.robot_cmd.position = np.subtract(self.robot_cmd.position, tuple(self._zero_offset))

        # Set max and min effort limits
        effort_limit = self.get_parameter(self.get_parameter("/snake_type") + "/max_torque")
        if effort_limit=='': # Setting default value
            effort_limit=1.5

        self.robot_cmd.effort_limit_max = effort_limit
        self.robot_cmd.effort_limit_min = -effort_limit

        self.send_joint_commands()
    
    def sort_joint_commands(self, unsorted_cmds, names_list=None):
        r"""Rearranges the joint commands to match order of the module names as provided in `HEBIROSWrapper`'s
        instantiation.

        Rearranging is done by matching the provided `names_list` to the saved `module_names` attribute. If not provided,
        no commands will be executed and warning will be issued. NaNs are returned if `unsorted_cmds` does not match
        the number of modules.

        Args:
            unsorted_cmds (list): List of joint commands.
            names_list (list): List of module names.

        Returns:
            sorted_cmds (list): List of joint commands sorted by module names.

        """
        if len(names_list) != self.num_modules:
            self.logwarn_throttle(
                5,
                f"Message from topic {self.joint_command_topic} either does not contain a joint names list or does not \
                    have correct number of entries. Not executing published commands.",
            )
            return self.num_modules * [np.nan]  # HEBI controller requires sending NaNs for no actions.
        elif len(unsorted_cmds) != self.num_modules:
            return self.num_modules * [np.nan]  # HEBI controller requires sending NaNs for no actions.
        else:
            joint_commands_with_names = dict(zip(names_list, unsorted_cmds))  # Pair module names with commands.
            # Rearrange commands to match saved module names.
            sorted_cmds = [joint_commands_with_names.get(module_name) for module_name in self.module_names]
        return sorted_cmds




def main(args=None):
    rclpy.init(args=args)
    node = HEBIControlROSWrapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
