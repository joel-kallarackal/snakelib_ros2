from snakelib_hebi.hebi_interface import HEBIROSWrapper
import sys
import time
import numpy as np
import rclpy
from scripts.camera_utils import to_img_msg
from scripts.streaming_utils import toggle_cam_stream, toggle_led_stream
from scripts.video_capture import VideoCapture
from rclpy.parameter import Parameter
import cv2

class CameraStreamerBase(HEBIROSWrapper):
    """Base class for defining shared methods across different cameras."""

    _VALID_CAM_NAMES = ["pinhole", "fisheye", "thermal", "led"]
    _VALID_CAM_STREAMS = ["pinhole", "fisheye", "thermal", "led"]
    EPSILON = sys.float_info.epsilon
    
    def __init__(self, node_name):
        # Establish communication with the HEBI modules.
        super().__init__(node_name)

        self.to_numpy_img = lambda x: np.frombuffer(x.data, dtype=np.uint8).reshape(x.height, x.width, -1)

        self.declare_parameter("led",Parameter.Type.BOOL)

        # Set the camera names
        self.cam_names = []

    def run_loop(self):
        r"""Publishes the required processed images."""
        self.update_feedback()

        for cam_name in self.cam_names:
            self._publish_stream(cam_name)

    def _publish_stream(self, cam_name):
        """Publishes the specified cam stream if cam img exists and is valid and cam stream is requested.

        Args:
            cam_name (str): Name of the camera stream to publish.
        """
        if cam_name not in self._VALID_CAM_STREAMS:
            self.logwarn_throttle(10, f"Invalid cam name: {cam_name}. Nothing published.")
        if getattr(self, cam_name) or cam_name == "processed_overlay":
            if hasattr(self, f"{cam_name}_img"):
                img = getattr(self, f"{cam_name}_img")
                pub = getattr(self, f"{cam_name}_cam_pub")
                if img is not None:
                    pub.publish(img)

    def logwarn_throttle(self, period_sec: float, msg: str, key='default'):
        """
        Logs a warning message at most once every `period_sec` seconds.

        Args:
            period_sec (float): Minimum seconds between log messages.
            msg (str): Warning message to log.
            key (str): Identifier to track multiple throttled messages separately.
        """
        now = time.time()
        last_time = self._last_warn_time.get(key, 0)
        if now - last_time >= period_sec:
            self.get_logger().warn(msg)
            self._last_warn_time[key] = now

    def _image_callback(self, msg, cam_name):
        img = self.to_numpy_img(msg)
        setattr(self, f"_{cam_name}_img", img)

    def update_feedback(self):
        r"""Populates respective attributes with updated sensor readings."""
        # Get raw camera feed from the snake head.
        feedback = self.get_feedback()

        # Convert the processed images to ROS Image messages.
        t_now = time.time()

        for k, v in feedback.items():
            setattr(self, k, to_img_msg(v, t=t_now))
    
    def get_feedback(self):
        r"""Returns raw feed from the snake head cameras.

        Returns:
            feedback (dict): Dictionary containing the feedback from the snake head cameras.
                - rgb_img (list): RGB camera image.
                - thermal_img (list): Thermal camera image.
        """
        feedback = {f"{cam_name}_img": self._safe_img_read(cam_name) for cam_name in self.cam_names}

        return feedback
    
    def _safe_img_read(self, cam_name):
        r"""Safely tries to read http camera feed. Returns None if feed doesn't exist or invalid cam name provided.

        Args:
            cam_name (str): Attribute name of the camera stream to read.

        Returns:
            img (np.ndarray): Numpy array image
        """
        img = None

        if not getattr(self, f"_{cam_name}_status"):
            return img

        if cam_name not in self._VALID_CAM_NAMES:
            self.logwarn_throttle(10, f"Invalid cam name: {cam_name}")
            return None
        else:
            img = getattr(self, f"_{cam_name}_reader")()
            if img is None:
                self.get_logger().warn(f"problem with {cam_name}. Attempting restart.")
                setattr(self, f"_{cam_name}_status", False)
        return img
    
    def _update_usb_cam_status(self, cam_name, address):
        r"""Checks the latest user-set status of the camera and toggles the camera stream if required.

        Args:
            cam_name (str): Name of the camera stream to publish.
            address (str): Http address of the camera stream.
        """

        # Use below declaration only if parameters not launched via a launch file
        # self.declare_parameter(f"stream_{cam_name}",False)
        self.declare_parameter(f"stream_{cam_name}",Parameter.Type.BOOL)
        status = self.get_parameter(f"stream_{cam_name}")
        if not hasattr(self, f"_{cam_name}_status"):
            setattr(self, f"_{cam_name}_status", not status)
        if status and not getattr(self, f"_{cam_name}_status"):
            toggle_cam_stream(f"{cam_name}", True)
            setattr(self, f"_{cam_name}_status", True)
            time.sleep(1) # Allow some time for the stream to start.
            try:
                cap = cv2.VideoCapture(f"{address}")
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
                streamer = VideoCapture(cap)
                setattr(self, f"{cam_name}_cap", streamer)

            except Exception:
                setattr(self, f"_{cam_name}_status", False)

        if not status and getattr(self, f"_{cam_name}_status"):
            toggle_cam_stream(f"{cam_name}", False)
            setattr(self, f"_{cam_name}_status", False)
            if hasattr(self, f"{cam_name}_cap"):
                getattr(self, f"{cam_name}_cap").release()
        return getattr(self, f"_{cam_name}_status")

    def close(self):
        for cam_name in self.cam_names:
            if hasattr(self, f"{cam_name}_cap"):
                getattr(self, f"{cam_name}_cap").release()

    @property
    def led(self):
        r"""Checks the latest user-set status of the LED stream and toggles the LED stream if required."""
        status = self.get_parameter("led")
        if not hasattr(self, "_led_status"):
            setattr(self, "_led_status", not status)
        if status and not self._led_status:
            toggle_led_stream("led", True)
            self._led_status = True
        if not status and self._led_status:
            toggle_led_stream("led", False)
            self._led_status = False
        return getattr(self, "_led_status")

        
        