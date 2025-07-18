import rclpy
from rclpy.node import Node
import hebi
from rclpy.duration import Duration
import os
from rclpy.parameter import Parameter
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class HEBIROSWrapper(Node):
    def __init__(self, node_name="hebi_ros_wrapper", **kwargs):
        super().__init__(node_name, **kwargs)
        
        params_path1 = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'snake_params.yaml')
        
        with open(params_path1, "r") as file:
            data = yaml.safe_load(file)

        params_path2 = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'launch_params.yaml')
        
        with open(params_path2, "r") as file:
            self._snake_type = yaml.safe_load(file).get("snake_type")
        
        self._snake_param = data.get("command_manager").get("ros__parameters").get(f"{self._snake_type}", {})

        # Get robot specific configs.
        self.robot_name = self._snake_type
        robot_configs =  self._snake_param

        # Get the module names.
        self.module_names = robot_configs.get("module_names", [])
        self.max_torque = robot_configs.get("max_torque", 7)  # Nm

        lookup_success = False
        n_lookup_tries = 0
        while (lookup_success is False) and n_lookup_tries < 5:
            lookup_success = self.lookup_modules()
            n_lookup_tries += 1
            self.get_logger().info(f"Look try: {n_lookup_tries}, success: {lookup_success}")
        if not lookup_success:
            raise Exception("Robot not detected or robot initialized not complete.")

        self.num_modules = self.robot.size

        self.feedback_frequency = data.get("command_manager").get("ros__parameters").get("hebi_feedback_frequency")  # Hz
        self.control_frequency = data.get("command_manager").get("ros__parameters").get("hebi_control_frequency")  # Hz
        self.robot.command_lifetime = data.get("command_manager").get("ros__parameters").get("hebi_command_lifetime") # ms

        # Issue a warning should the loop frequence be too low.
        if min(self.feedback_frequency, self.control_frequency) < (1000 / self.robot.command_lifetime):
            self.get_logger().warn(
                f"Send commands faster than command lifetime ({self.robot.command_lifetime} ms) to avoid missing \
                    commands (uncontrolled intervals). \n Control freq: {self.control_frequency} Hz, Feedback freq: \
                    {self.feedback_frequency} Hz",
            )
        
    def lookup_modules(self):
        # Find online modules.
        self.lookup = hebi.Lookup()
        self.get_clock().sleep_for(Duration(seconds=2.0))  # Allow some time for the lookup procedure to find online modules.

        self.get_logger().info("--Looking up modules--")
        msg = ""
        for entry in self.lookup.entrylist:
            msg += f"{entry.family} | {entry.name}"
        self.get_logger().info(msg)

        # Initialize robot handle.
        self.robot = self.lookup.get_group_from_names("*", self.module_names)
        lookup_success = True if self.robot is not None else False
        return lookup_success
    
    def start_log(self):
        cwd = os.path.dirname(os.path.abspath(__file__))
        log_dir = cwd + "/../../../logs/"
        if not os.path.isdir(log_dir):
            os.mkdir(log_dir)
        self.log_location = self.robot.start_log(directory=log_dir)
    
    def stop_log(self):
        self.robot.stop_log()

    def start(self):
        r"""Run loop until ros is not shutdown."""
        rate = self.create_rate(self.loop_rate)

        while rclpy.ok():
            self.run_loop()

            rclpy.spin_once(self)
            rate.sleep()

    def run_loop(self):
        pass

