import rclpy

from script.camera_interface import CameraStreamerBase
from sensor_msgs.msg import Image
from script.camera_utils import to_img_msg
import time

import yaml
import os
from ament_index_python.packages import get_package_share_directory

class PinholeCameraStreamer(CameraStreamerBase):
    """Driver node that publishes the camera data from the snake head."""

    def __init__(self):
        # Establish communication with the HEBI modules.
        super().__init__('pinhole_node')

        params_path1 = os.path.join(get_package_share_directory('snakelib_camera'), 'param', 'camera_params.yaml')
        
        with open(params_path1, "r") as file:
            self.data = yaml.safe_load(file)
        
        # Initialize running attributes.
        self._pinhole_img = None  # Raw images buffer

        # Initialize required publishers.
        self.pinhole_cam_pub = self.create_publisher(Image, '/pinhole_cam/image_raw', 1)


        # Set loop rate as camera FPS.
        self.loop_rate = self.data.get("camera_frequency")  # FPS

        self.cam_address = self.data.get("pinhole_cam_address")

        self.get_logger().info(f"Pinhole cam status: {self.pinhole}")

        self.cam_names = ["pinhole", "led"]  # LED is here since this cam (node) will be on most of the time.

    def update_feedback(self):
        r"""Populates respective attributes with updated sensor readings."""
        # Get raw camera feed from the snake head.
        feedback = self.get_feedback()

        # Convert the processed images to ROS Image messages.
        self.pinhole_img = to_img_msg(feedback["pinhole_img"], time.time())
    
    def get_feedback(self):
        r"""Returns raw feed from the snake head cameras.

        Returns:
            feedback (dict): Dictionary containing the feedback from the snake head cameras.
                - rgb_img (list): RGB camera image.
                - thermal_img (list): Thermal camera image.
        """
        feedback = {
            "pinhole_img": self._safe_img_read("pinhole"),
        }

        return feedback
    
    def _pinhole_reader(self):
        _, img = getattr(self, "pinhole_cap").read()
        return img

    @property
    def pinhole(self):
        status = self._update_usb_cam_status("pinhole", self.cam_address)
        return status

class FisheyeCameraStreamer(CameraStreamerBase):
    """Driver node that publishes the camera data from the snake head."""

    def __init__(self):
        # Establish communication with the HEBI modules.
        super().__init__('fisheye_node')

        # Initialize running attributes.
        self._fisheye_img = None  # Raw images buffer

        # Initialize required publishers.
        self.fisheye_cam_pub = self.create_publisher(Image, '/fisheye_cam/image_raw', 1)

        # Set loop rate as camera FPS.
        self.loop_rate = self.data.get("camera_frequency", 30.0)  # FPS\

        self.cam_address = self.data.get("fisheye_cam_address")

        self.get_logger().info(f"Fisheye cam status: {self.fisheye}")

        self.cam_names = ["fisheye"]
    
    def update_feedback(self):
        r"""Populates respective attributes with updated sensor readings."""
        # Get raw camera feed from the snake head.
        feedback = self.get_feedback()

        # Convert the processed images to ROS Image messages.
        self.fisheye_img = to_img_msg(feedback["fisheye_img"], time.time())

    def get_feedback(self):
        r"""Returns raw feed from the snake head cameras.

        Returns:
            feedback (dict): Dictionary containing the feedback from the snake head cameras.
                - rgb_img (list): RGB camera image.
                - thermal_img (list): Thermal camera image.
        """
        feedback = {
            "fisheye_img": self._safe_img_read("fisheye"),
        }

        return feedback

    def _fisheye_reader(self):
        _, img = getattr(self, "fisheye_cap").read()
        return img

    @property
    def fisheye(self):
        status = self._update_usb_cam_status("fisheye", self.cam_address)
        return status
