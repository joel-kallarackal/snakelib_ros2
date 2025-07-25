#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import csv

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug')
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/snake/joint_states',  
            self.joint_states_callback,
            10
        )
        self.joint_states_sub  # prevent unused variable warning

        self.joint_commands_sub = self.create_subscription(
            JointState,
            '/snake/joint_commands',  
            self.joint_commands_callback,
            10
        )
        self.joint_commands_sub

        self.acheived_pos = []
        self.commanded_pos = []

    def joint_states_callback(self, msg):
        pos = msg.position
        pos.append(time.time())
        self.acheived_pos.append(pos)


    def joint_commands_callback(self, msg):
        pos = msg.position
        pos.append(time.time())
        self.commanded_pos.append(pos)

    def shutdown(self):
        t = int(time.time())
        filename_states = f"src/snakelib_debug/logs/joint_states_{t}.csv"
        filename_commands = f"src/snakelib_debug/logs/joint_commands_{t}.csv"
        
        with open(filename_states, mode='w', newline='') as file1:
            writer = csv.writer(file1)
            writer.writerows(self.acheived_pos)

        self.get_logger().info(f"Joint states written to {filename_states}")

        with open(filename_commands, mode='w', newline='') as file2:
            writer = csv.writer(file2)
            writer.writerows(self.commanded_pos)

        self.get_logger().info(f"Joint commands written to {filename_commands}")

        


   
def main(args=None):
    rclpy.init(args=args)
    node = DebugNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
