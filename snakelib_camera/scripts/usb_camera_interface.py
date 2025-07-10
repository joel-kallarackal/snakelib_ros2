import rclpy

from scripts.camera_interface import CameraStreamerBase
from sensor_msgs.msg import Image
from rclpy.parameter import Parameter
from scripts.camera_utils import to_img_msg
import time

class PinholeCameraStreamer(CameraStreamerBase):
    """Driver node that publishes the camera data from the snake head."""

    def __init__(self):
        # Establish communication with the HEBI modules.
        super().__init__('pinhole_node')
        
        # Initialize running attributes.
        self._pinhole_img = None  # Raw images buffer

        # Initialize required publishers.
        self.pinhole_cam_pub = self.create_publisher(Image, '/pinhole_cam/image_raw', 1)


        # Set loop rate as camera FPS.
        self.declare_parameter("camera_frequency", Parameter.Type.DOUBLE)
        self.loop_rate = self.get_parameter("camera_frequency")  # FPS\

        self.declare_parameter("pinhole_cam_address", Parameter.Type.STRING)
        self.cam_address = self.get_parameter("pinhole_cam_address")

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
        self.declare_parameter("camera_frequency", Parameter.Type.DOUBLE)
        self.loop_rate = self.get_parameter("camera_frequency", 30.0)  # FPS\

        self.declare_parameter("camera_frequency", Parameter.Type.STRING)
        self.cam_address = self.get_parameter("fisheye_cam_address")

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
