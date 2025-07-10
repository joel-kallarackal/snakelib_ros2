"""Utility functions to start/stop streaming from the RPI (snake head) server.
"""
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from snakelib_srvs.srv import ToggleStream

def execute_script(password, ip, base_dir, script_name, script_args=None):
    """SSH into the RPI, trigger the relevant scripts to toggle requested streaming.

    Args:
        password (str): password for the head module
        ip (str): IP address for the head module
        base_dir (str): base directory for the script to be executed
        script_name (str): name of the script to be executed
        script_args (optional): arguments required by the script
    """
    if ".sh" not in script_name:
        script_name += ".sh"

    # Bypass prompt for fingerprint addition and strick host key checking
    login = f"sshpass -p {password} ssh -o 'StrictHostKeyChecking no' root@{ip}"
    pos_login_execution = f"cd {base_dir} && bash {script_name} {script_args}"

    cmd = f'{login} "{pos_login_execution}"'
    os.system(cmd)

def wait_for_service(service_name: str, timeout_sec: float = 1.0):
    rclpy.init()
    if service_name=="toggle_stream":
        node = rclpy.create_node('toggle_stream_client')
        client = node.create_client(ToggleStream, service_name)
        while not client.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().info(f'Waiting for service "{service_name}"...')
    
    return client

def toggle_cam_stream(stream_name, on, max_attempts=2):
    """
    Try to toggle camera stream at snake head server `max_attempts` times before giving up.

    Args:
        stream_name (str): Name of the stream to be toggled
        on (bool): Desired status of the stream (True = 'on', False = 'off')
        max_attempts (int): Max attempts before giving up

    Returns:
        bool: True if successful, False otherwise
    """
    rclpy.init()
    node = rclpy.create_node('toggle_cam_stream_client_node')

    client = node.create_client(ToggleStream, 'toggle_stream')

    mode = 'on' if on else 'off'

    # Wait for service to become available
    attempt = 0
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for toggle_stream service...')
        # attempt += 1
        # if attempt >= max_attempts:
        #     node.get_logger().warn(f"Service not available after {max_attempts} attempts.")
        #     node.destroy_node()
        #     rclpy.shutdown()
        #     return False

    # Try calling the service up to max_attempts times
    for attempt in range(1, max_attempts + 1):
        request = ToggleStream.Request()
        request.stream_name = stream_name
        request.on = on

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            node.get_logger().info(f"Status ({stream_name}): {mode}")
            node.destroy_node()
            rclpy.shutdown()
            return True
        else:
            node.get_logger().warn(
                f"[ServiceException] Failed attempt [{attempt}/{max_attempts}] to turn {stream_name}: {mode}"
            )

    node.destroy_node()
    rclpy.shutdown()
    return False

def toggle_led_stream(stream_name, on, max_attempts=2):
    """
    Try to toggle camera stream at snake head server `max_attempts` times before giving up.

    Args:
        stream_name (str): Name of the stream to be toggled
        on (bool): Desired status of the stream (True = 'on', False = 'off')
        max_attempts (int): Max attempts before giving up

    Returns:
        bool: True if successful, False otherwise
    """
    rclpy.init()
    node = rclpy.create_node('toggle_led_stream_client_node')

    client = node.create_client(ToggleStream, 'toggle_stream')

    mode = 'on' if on else 'off'

    # Wait for service to become available
    attempt = 0
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for toggle_stream service...')
        # attempt += 1
        # if attempt >= max_attempts:
        #     node.get_logger().warn(f"Service not available after {max_attempts} attempts.")
        #     node.destroy_node()
        #     rclpy.shutdown()
        #     return False

    # Try calling the service up to max_attempts times
    for attempt in range(1, max_attempts + 1):
        request = ToggleStream.Request()
        request.stream_name = stream_name
        request.on = on

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            node.get_logger().info(f"Status ({stream_name}): {mode}")
            node.destroy_node()
            rclpy.shutdown()
            return True
        else:
            node.get_logger().warn(
                f"[ServiceException] Failed attempt [{attempt}/{max_attempts}] to turn {stream_name}: {mode}"
            )

    node.destroy_node()
    rclpy.shutdown()
    return False

