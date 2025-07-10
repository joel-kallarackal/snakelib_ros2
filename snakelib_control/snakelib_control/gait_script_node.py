#!/usr/bin/env python3

""" Node that publishes a sequence of gaits to run through for testing.

Uses the default gait parameters.
"""

from sensor_msgs.msg import JointState
from snakelib_msgs.msg import SnakeCommand
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

def main():
    try:
        rclpy.init()
        node = Node("gait_script_node")
        pub = node.create_publisher(SnakeCommand, 'snake/command', 10)
        
        loop_rate = 10
        rate = node.create_rate(loop_rate)

        seconds_per_gait = 6
        loops_per_gait = loop_rate*seconds_per_gait

        commands = ["lateral_undulation","home","linear_progression","hold_position","rolling","sidewinding","slithering","turn_in_place"]

        while rclpy.ok():

            for command in commands:
                for x in range(loops_per_gait):
                    cmd = SnakeCommand()
                    cmd.command_name = command
                    pub.publish(cmd)
                    
                    rclpy.spin_once(node)
                    rate.sleep()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
         


if __name__ == "__main__":
    main()
