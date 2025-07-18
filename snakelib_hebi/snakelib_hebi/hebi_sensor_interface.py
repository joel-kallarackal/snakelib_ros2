import rclpy
import hebi
import numpy as np
from sensor_msgs.msg import JointState
import time
from rclpy.parameter import Parameter
from snakelib_msgs.msg import HebiSensors
from rclpy.executors import ExternalShutdownException
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from snakelib_hebi.hebi_interface import HEBIROSWrapper

class HEBISensingROSWrapper(HEBIROSWrapper):
    """Driver node that publishes sensor (joint angle, joint velocity, effort, linear acceleration,
    angular velocity, and orientation) data from the HEBI modules."""

    def __init__(self):
        # Establish communication with the HEBI modules.
        super().__init__('hebi_sensor_interface')

        self.start_log()

        # Initialize required publishers.
        self.hebi_sensors_pub = self.create_publisher(HebiSensors, '/hebi_sensors/data', 10)
        self.joint_state_pub = self.create_publisher(JointState, "/snake/joint_states", 10)

        # Initialize feedback message buffers.
        self.hebi_sensors = HebiSensors()
        self.joint_state = JointState()

        # Set loop rate as feedback frequency.
        self.loop_rate = self.feedback_frequency
        self.robot_feedback = hebi.GroupFeedback(self.num_modules)

        params_path1 = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'snake_params.yaml')
        
        with open(params_path1, "r") as file:
            data = yaml.safe_load(file)

        params_path2 = os.path.join(get_package_share_directory('snakelib_control'), 'param', 'launch_params.yaml')
        
        with open(params_path2, "r") as file:
            self._snake_type = yaml.safe_load(file).get("snake_type")
        
        self._snake_param = data.get("command_manager").get("ros__parameters").get(f"{self._snake_type}", {})

        self._zero_offset = self._snake_param.get("zero_offset")
    
    def run_loop(self):
        r"""Send commands regularly until shutdown."""
        self.update_feedback()
        self.hebi_sensors_pub.publish(self.hebi_sensors)
        self.joint_state_pub.publish(self.joint_state)

    def update_feedback(self):
        r"""Populates respective attributes with updated sensor readings."""
        # Read raw feedback from the module sensors.
        feedback = self.get_feedback()

        # Populate hebi sensors message with feedback.
        self.hebi_sensors.header.stamp = self.get_clock().now().to_msg()
        self.hebi_sensors.name = self.module_names
        self.hebi_sensors.position = list(map(float, feedback["position"]))
        self.hebi_sensors.velocity = list(map(float, feedback["velocity"]))
        self.hebi_sensors.effort = list(map(float, feedback["effort"]))

        self.hebi_sensors.lin_acc.x = list(map(float, feedback["lin_acc"][:, 0]))
        self.hebi_sensors.lin_acc.y = list(map(float, feedback["lin_acc"][:, 1]))
        self.hebi_sensors.lin_acc.z = list(map(float, feedback["lin_acc"][:, 2]))


        self.hebi_sensors.ang_vel.x = list(map(float, feedback["ang_vel"][:, 0]))
        self.hebi_sensors.ang_vel.y = list(map(float, feedback["ang_vel"][:, 1]))
        self.hebi_sensors.ang_vel.z = list(map(float, feedback["ang_vel"][:, 2]))

        self.hebi_sensors.orientation.x = list(map(float, feedback["orientation"][:, 0]))
        self.hebi_sensors.orientation.y = list(map(float, feedback["orientation"][:, 1]))
        self.hebi_sensors.orientation.z = list(map(float, feedback["orientation"][:, 2]))
        self.hebi_sensors.orientation.w = list(map(float, feedback["orientation"][:, 3]))

        self.joint_state.header = self.hebi_sensors.header
        self.joint_state.name = self.module_names
        self.joint_state.position = self.hebi_sensors.position
        self.joint_state.velocity = self.hebi_sensors.velocity
        self.joint_state.effort = self.hebi_sensors.effort

        # Perform filtering on the raw data.
        # TODO: In case filtering is required, add here.
    
    def get_feedback(self):
        r"""Returns raw feedback from the module sensors. Uses old feedback, should new feedback be not available.

        Returns:
            feedback (dict): Dictionary containing the feedback from the module sensors.
                - joint positions (list): List of joint positions.
                - joint velocities (list): List of joint velocities.
                - joint efforts (list): List of joint efforts.
                - linear accelerations (list): List of 3D linear accelerations.
                - angular velocities (list): List of 3D angular velocities.
                - orientation (list): List of quaternion orientation.
        """
        fbk_old = self.robot_feedback
        self.robot_feedback = self.robot.get_next_feedback(reuse_fbk=self.robot_feedback)

        if self.robot_feedback is None:
            self.robot_feedback = fbk_old

        feedback = {
            "position": self.robot_feedback.position + self._zero_offset,
            "velocity": self.robot_feedback.velocity,
            "effort": self.robot_feedback.effort,
            "lin_acc": self.robot_feedback.accelerometer,
            "ang_vel": self.robot_feedback.gyro,
            "orientation": self.robot_feedback.orientation,
        }

        return feedback



def main():
    rclpy.init()
    hebi_sensor_interface_node = HEBISensingROSWrapper()

    try:
        hebi_sensor_interface_node.start()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        hebi_sensor_interface_node.stop_log()
        hebi_sensor_interface_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
