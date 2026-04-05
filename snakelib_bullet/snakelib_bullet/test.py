import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

class SnakeCommander(Node):
    def __init__(self):
        super().__init__('snake_commander')
        self.pub = self.create_publisher(JointState, '/snake/joint_commands', 10)
        self.joint_names = ["SA001__MoJo",
                            "SA002__MoJo",
                            "SA003__MoJo",
                            "SA004__MoJo",
                            "SA005__MoJo",
                            "SA006__MoJo",
                            "SA007__MoJo",
                            "SA008__MoJo",
                            "SA009__MoJo",
                            "SA010__MoJo",
                            "SA011__MoJo",
                            "SA012__MoJo",
                            "SA013__MoJo",
                            "SA014__MoJo",
                            "SA015__MoJo",
                            "SA016__MoJo"
                            ]
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_command)
        self.t = 0.0

    def publish_command(self):
        msg = JointState()
        msg.name = self.joint_names

        # Example: sinusoidal gait
        amplitude = 0.5
        frequency = 0.5
        msg.position = [amplitude * np.sin(2*np.pi*frequency*self.t + i*0.5) for i in range(16)]

        self.pub.publish(msg)
        self.t += self.timer_period

def main():
    rclpy.init()
    node = SnakeCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()