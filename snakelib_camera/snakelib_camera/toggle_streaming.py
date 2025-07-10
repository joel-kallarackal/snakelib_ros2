from __future__ import print_function

import rclpy
from rclpy.node import Node


from snakelib_srvs.srv import ToggleStream, ToggleStreamResponse
from scripts.streaming_utils import execute_script

PASSWORD = "fa"
RPI_IP = "192.168.8.132"
CODE_DIR = "/root/reu-snake-head-vision/mlx90641-driver-master/scripts/"


def toggle_stream_fn(req):
    # Execute the appropriate remote server (snake head) command here
    mode = "launch" if req.on else "stop"
    script_name = f"{mode}_background_stream"
    script_args = f"{req.stream_name}_sender"
    password = PASSWORD
    rpi_ip = RPI_IP
    code_dir = CODE_DIR
    success = execute_script(password, rpi_ip, code_dir, script_name, script_args)
    return ToggleStreamResponse(success)


def toggle_led_fn(req):
    # Execute the appropriate remote server (snake head) command here
    mode = "on" if req.on else "off"
    script_name = f"led_{mode}"
    script_args = ""
    password = PASSWORD
    rpi_ip = RPI_IP
    code_dir = CODE_DIR
    success = execute_script(password, rpi_ip, code_dir, script_name, script_args)
    return ToggleStreamResponse(success)


def toggle_streaming():
    rclpy.init()
    node = Node("toggle_streaming")

    node.create_service(ToggleStream, "toggle_stream", toggle_stream_fn)
    node.create_service(ToggleStream, "toggle_led", toggle_led_fn)
    node.get_logger().info("Camera stream toggle available.")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


def main():
    toggle_streaming()

if __name__ == "__main__":
    main()
