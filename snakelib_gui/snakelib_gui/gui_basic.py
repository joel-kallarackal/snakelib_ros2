import tkinter as tk

import numpy as np
from PIL import Image as PIL_Image
from PIL import ImageTk
from sensor_msgs.msg import Image
from rclpy.parameter import Parameter


class GUIBasic:
    def __init__(self, node):

        self.node = node

        # Define GUI Layout
        self.create_gui()

        # Image Subscriber
        self.image_sub = self.create_subscription(
            Image,
            "processed_overlay_cam/image_raw",
            self.camera_feed_cb,
            10)
        self.image_sub
        
        # Start running the GUI
        self.window.mainloop()

    def create_gui(self):
        """
        Creates a Tkinter GUI window and populates them with the necessary buttons and camera feed windows
        """

        # GUI Window layout
        self.window = tk.Tk()
        self.window.geometry("1460x1080")
        self.window.title("Snake Camera Feed")

        # Toggle Buttons
        self.toggle_led_button = tk.Button(
            self.window, text="Toggle LED", command=lambda: self.toggle_param("/led"), font=("TkDefaultFont", 12, "bold")
        )
        self.toggle_led_button.pack(expand=True, fill="both")

        self.toggle_thermal_button = tk.Button(
            self.window,
            text="Toggle Thermal Overlay",
            command=lambda: self.toggle_param("/stream_overlay"),
            font=("TkDefaultFont", 12, "bold"),
        )
        self.toggle_thermal_button.pack(expand=True, fill="both")

        self.toggle_roll_button = tk.Button(
            self.window,
            text="Toggle Roll Compensation",
            command=lambda: self.toggle_param("/rotate_image"),
            font=("TkDefaultFont", 12, "bold"),
        )
        self.toggle_roll_button.pack(expand=True, fill="both")

        # Camera Feed Window
        self.camera_frame = tk.Label(self.window, width=1280, height=960, background="black")  # Modify Height and width
        self.camera_frame.pack(expand=True, fill="both")

    def toggle_param(self, param_name):
        """
        Inverts the appropriate parameter on the ROS server
        This is queried by the camera node to change camera settings
        """

        self.node.declare_parameter(param_name, Parameter.Type.BOOL)
        param_status = self.node.get_parameter(param_name)
        self.node.set_parameters([Parameter(param_name, Parameter.Type.BOOL, not param_status)])

    def camera_feed_cb(self, msg):
        """
        Parses the Image datatype into a numpy array and then into a Tkinter Image format
        This photo is then sent to the camera feed window
        """

        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)[:, :, ::-1]  # cv2 uses BGR format, convert to RGB
        image = ImageTk.PhotoImage(image=PIL_Image.fromarray(image, mode="RGB").resize((msg.height * 2, msg.width * 2)))
        self.camera_frame.configure(image=image)
        self.camera_frame.photo_image = image

    def close(self):
        """
        Close the GUI on signal interrruption
        """
        self.window.destroy()
