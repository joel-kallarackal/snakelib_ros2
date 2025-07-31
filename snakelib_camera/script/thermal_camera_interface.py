import rclpy

from script.camera_interface import CameraStreamerBase
from sensor_msgs.msg import Image
from script.streaming_utils import toggle_cam_stream
import time
import numpy as np
import socket
import struct

import yaml
import os
from ament_index_python.packages import get_package_share_directory

class ThermalCameraStreamer(CameraStreamerBase):
    """Driver node that publishes the camera data from the snake head."""

    def __init__(self):
        # Establish communication with the HEBI modules.
        super().__init__('thermal_node')

        # Initialize running attributes.
        self.thermal_img = self._thermal_img = None

        params_path1 = os.path.join(get_package_share_directory('snakelib_camera'), 'param', 'camera_params.yaml')
        
        with open(params_path1, "r") as file:
            self.data = yaml.safe_load(file)

        # Initialize camera parameters.
        self.port = self.data.get("port")
        self.ip = self.data.get("ip")

        self.min_temp = self.data.get("min_temp")
        self.max_temp = self.data.get("max_temp")
        self.img_rows = self.data.get("thermal_img_rows")
        self.img_cols = self.data.get("thermal_img_cols")
        self.byte_size = self.data.get("thermal_byte_size")
        self.img_size = (self.img_rows * self.img_cols) * self.byte_size

        self.thermal_cam_pub = self.create_publisher(Image, '/thermal_cam/image_raw', 1)
        self.loop_rate = self.data.get("thermal_camera_frequency")  # FPS

        self.cam_names = ["thermal"]

        self.get_logger().info(f"Thermal cam status: {self.thermal}")

    def _thermal_reader(self):
        image_np = np.zeros((self.img_rows, self.img_cols))

        image = self.socket.recv(self.img_size)
        iter_obj = struct.iter_unpack("f", image)

        try:
            for i in range(0, self.img_rows):
                for j in range(0, self.img_cols):
                    image_np[i][j] = next(iter_obj)[0]

        except StopIteration:
            return None

        thermal_np_img = image_np.reshape(self.img_rows, self.img_cols).astype(np.float32)
        thermal_img = np.zeros((self.img_rows, self.img_cols, 3), dtype="uint8")
        thermal_greyscale = np.zeros((self.img_rows, self.img_cols), dtype="uint8")

        for y in range(len(thermal_np_img[0])):
            for x in range(len(thermal_np_img)):
                # rgb value (use for raw visualization)
                thermal_img[x][y] = self._to_rgb(thermal_np_img[x][y], self.min_temp, self.max_temp)
                # pixel value (use for overlay)
                pixel = (thermal_np_img[x][y] - self.min_temp) * (255 / (self.max_temp - self.min_temp)) + 0.5
                pixel = min(255, max(0, pixel))
                thermal_greyscale[x][y] = pixel
        thermal_img = np.rot90(thermal_img)
        thermal_greyscale = np.rot90(thermal_greyscale)
        return thermal_img
    
    def _to_rgb(self, raw_value, min_temp, max_temp):
        r"""Converts raw thermal sensor readings to RGB values

        Args:
            raw_values (np.ndarray): Raw sensor readings
            min_temp (float): Minimum temperature parameter for conversion
            max_temp (float): Maximum temperature parameter for conversion
        """
        value = np.clip(np.nan_to_num(raw_value), self.min_temp, self.max_temp)
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # [BLUE, GREEN, RED]

        i_f = float(value - min_temp) / float(max_temp - min_temp) * (len(colors) - 1)
        i, f = int(i_f // 1), i_f % 1  # Split into whole & fractional parts.
        if f < self.EPSILON:
            return colors[i]
        else:
            (r1, g1, b1), (r2, g2, b2) = colors[i], colors[i + 1]
            r_value, g_value, b_value = int(r1 + f * (r2 - r1)), int(g1 + f * (g2 - g1)), int(b1 + f * (b2 - b1))
            return [r_value, g_value, b_value]

    @property
    def thermal(self):
        r"""Checks for the latest set status of thermal streaming and toggles stream if required."""
        status = self.data.get("stream_thermal", False)
        if not hasattr(self, "_thermal_status"):
            setattr(self, "_thermal_status", not status)
        if status and not getattr(self, "_thermal_status"):
            toggle_cam_stream("thermal", True)
            setattr(self, "_thermal_status", True)
            time.sleep(0.2)  # Allow some time for the stream to start.
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.socket.connect((self.ip, self.port))
            except Exception:
                setattr(self, "_thermal_status", False)
        if not status and getattr(self, "_thermal_status"):
            toggle_cam_stream("thermal", False)
            setattr(self, "_thermal_status", False)
            if hasattr(self, "socket"):
                self.socket.close()
                delattr(self, "socket")
        return self._thermal_status