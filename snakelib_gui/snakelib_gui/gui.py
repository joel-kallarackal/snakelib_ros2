#!/usr/bin/env python3
from __future__ import division, print_function

import os
import signal
import socket
import subprocess
from datetime import datetime
from time import sleep
from typing import List

import rclpy
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PIL.ImageQt import ImageQt
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
from python_qt_binding import QtNetwork
from rcl_interfaces.msg import Log
from rviz import bindings as rviz
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter

class Worker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def run(self):
        """Long-running task."""
        while rclpy.ok():
            sleep(1 / 15)
            self.progress.emit(1)
        self.finished.emit()

def toggle_param(node, param_name):
    node.declare_parameter(param_name)
    param_status = node.get_parameter(param_name).value
    node.set_parameters([rclpy.parameter.Parameter(param_name, rclpy.Parameter.Type.BOOL, not param_status)])

class SignalWakeupHandler(QtNetwork.QAbstractSocket):
    def __init__(self, parent=None):
        QtNetwork.QAbstractSocket.__init__(self, QtNetwork.QAbstractSocket.UdpSocket, parent)
        self.old_fd = None
        # Create a socket pair
        self.wsock, self.rsock = socket.socketpair()
        # Let Qt listen on the one end
        self.setSocketDescriptor(self.rsock.fileno())
        # And let Python write on the other end
        self.wsock.setblocking(False)
        self.old_fd = signal.set_wakeup_fd(self.wsock.fileno())
        # First Python code executed gets any exception from
        # the signal handler, so add a dummy handler first
        self.readyRead.connect(lambda: None)
        # Second handler does the real handling
        self.readyRead.connect(self._readSignal)

    def __del__(self):
        # Restore any old handler on deletion
        if self.old_fd is not None and signal and signal.set_wakeup_fd:
            signal.set_wakeup_fd(self.old_fd)

    def _readSignal(self):
        # Read the written byte.
        # Note: readyRead is blocked from occuring again until readData()
        # was called, so call it, even if you don't need the value.
        data = self.readData(1)
        # Emit a Qt signal for convenience
        self.signalReceived.emit(data[0])

    signalReceived = pyqtSignal(int)


class RosbagRecorder:
    """Object for recording custom topics to a rosbag file in a separate process.

    Reference:
        - https://github.com/biorobotics/roboTRAC/blob/538fa17ad285d57202a8c161d5014477b283f2ec/robo_trac/scripts/drivers_watchdog.py#L32
        - https://answers.ros.org/question/286871/how-record-rosbag-with-python/
    """

    def __init__(self, node , path: str, topics: List[str], exceptions: List[str] = None):
        self.node = node
        self.record_script = "rosrun rosbag record __name:='watchdog_record' -a --lz4 -O " + path
        if exceptions:
            self.record_script += " -x '("
            for regex in exceptions:
                self.record_script = self.record_script + "|" + regex
            self.record_script += ")'"
        for topic in topics:
            self.record_script += " " + topic
        rclpy.on_shutdown(self.stop_recording_handler)

        # Start recording.
        command = self.record_script
        try:
            self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=os.path.split(path)[0], executable="/bin/bash")
        except Exception as e:
            self.node.get_logger().warn(e)
            self.p = None

    def stop_recording_handler(self):
        """Terminates the process recording the rosbag file and logs info."""
        self.node.get_logger().info(self.node.get_name() + " stop recording.")
        self._terminate_ros_node("/watchdog_record")

    def _terminate_ros_node(self):
        """Terminates the process recording the rosbag file."""
        self.p.terminate()
        self.p.wait()

    @property
    def is_recording(self):
        """Returns the status of the rosbag recording process."""
        if self.p is None:
            return False
        return self.p.poll() is None


# class SnakeViz(QWidget):
#     def __init__(self, app, node):
#         QWidget.__init__(self)
#         self.frame = rviz.VisualizationFrame()
#         self.frame.setParent(self)
#         self.cam_frame = rviz.VisualizationFrame()
#         self.cam_frame.setParent(self)
#         self.node = node

#         self.frame.setSplashPath("")
#         self.cam_frame.setSplashPath("")
#         assert rclpy.ok()

#         self.frame.initialize()
#         self.cam_frame.initialize()

#         reader = rviz.YamlConfigReader()
#         config = rviz.Config()
#         path = get_package_share_directory('snakelib_gui')
#         reader.readFile(config, os.path.join(path, "rviz", "gui.rviz"))
#         self.frame.load(config)

#         cam_reader = rviz.YamlConfigReader()
#         cam_config = rviz.Config()
#         cam_reader.readFile(cam_config, os.path.join(path, "rviz", "cam.rviz"))
#         self.cam_frame.load(cam_config)

#         self.setWindowTitle(config.mapGetChild("Title").getValue())

#         self.frame.setMenuBar(None)
#         self.frame.setStatusBar(None)
#         self.frame.setHideButtonVisibility(False)
#         self.cam_frame.setMenuBar(None)
#         self.cam_frame.setStatusBar(None)
#         self.cam_frame.setHideButtonVisibility(False)

#         self.manager = self.frame.getManager()
#         self.manager.preUpdate.connect(self.update)

#         self.cam_manager = self.cam_frame.getManager()
#         self.cam_manager.preUpdate.connect(self.update)

#         self.robot_status = None

#         layout = QHBoxLayout()
#         sub_layout = QVBoxLayout()
#         # sub_layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

#         # Camera Feed
#         sub_layout.addWidget(self.cam_frame)

#         # Graphs
#         self.pos_figure = plt.figure(figsize=(4.0, 2.8))
#         self.pos_canvas = FigureCanvas(self.pos_figure)
#         sub_layout.addWidget(self.pos_canvas)

#         self.vel_figure = plt.figure(figsize=(4.0, 2.8))
#         self.vel_canvas = FigureCanvas(self.vel_figure)
#         sub_layout.addWidget(self.vel_canvas)

#         self.effort_figure = plt.figure(figsize=(4.0, 2.8))
#         self.effort_canvas = FigureCanvas(self.effort_figure)
#         sub_layout.addWidget(self.effort_canvas)

#         # Buttons
#         self.led_button = QPushButton("Toggle LED", self)
#         self.led_button.clicked.connect(lambda: toggle_param("/led"))

#         self.overlay_img_button = QPushButton("Toggle thermal overlay", self)
#         self.overlay_img_button.clicked.connect(lambda: toggle_param("/stream_overlay"))

#         self.rotate_img_button = QPushButton("Toggle roll compensation", self)
#         self.rotate_img_button.clicked.connect(lambda: toggle_param("/rotate_image"))

#         self.rosbag_button = QPushButton("Start recording bags", self)
#         self.rosbag_button.clicked.connect(self.ros_bags_record)

#         sub_layout.addWidget(self.led_button)
#         sub_layout.addWidget(self.overlay_img_button)
#         sub_layout.addWidget(self.rotate_img_button)
#         sub_layout.addWidget(self.rosbag_button)

#         # Status indicators
#         self.ui_robot_status = QLabel("Robot mode indicator")
#         sub_layout.addWidget(self.ui_robot_status)

#         self.ui_robot_msgs = QLabel("ROS log info msgs")
#         sub_layout.addWidget(self.ui_robot_msgs)

#         layout.addLayout(sub_layout, 1)

#         # Robot state
#         layout.addWidget(self.frame, 3)

#         self.manager.setParent(self)
#         self.cam_manager.setParent(self)

#         self.setLayout(layout)

#         self.robot_status_sub = self.node.create_subscription(
#             String,
#             "/current_status",
#             self.robot_status_cb,
#             5)
#         self.robot_status_sub  # prevent unused variable warning

#         self.log_info_sub = self.node.create_subscription(
#             Log,
#             "/rosout_agg",
#             self.ros_log_info_cb,
#             5)
#         self.log_info_sub  # prevent unused variable warning

#         self.joint_states_sub = self.node.create_subscription(
#             JointState,
#             "/snake/joint_states",
#             self.joint_states_cb,
#             1)
#         self.joint_states_sub  # prevent unused variable warning

#         self.thread = QThread()
#         self.worker = Worker()
#         # Step 4: Move worker to the thread
#         self.worker.moveToThread(self.thread)
#         # Step 5: Connect signals and slots
#         self.thread.started.connect(self.worker.run)
#         self.worker.finished.connect(self.thread.quit)
#         self.worker.finished.connect(self.worker.deleteLater)
#         self.thread.finished.connect(self.thread.deleteLater)

#     def ros_bags_record(self):
#         """Records the ros bags for topics specified in the configuration file"""
#         # Get state for rosbag record
#         if not hasattr(self, "_record_bags"):
#             self._record_bags = True
#         else:
#             self._record_bags = not self._record_bags

#         # Update button text
#         button_text = "Stop recording bags" if self._record_bags else "Start recording bags"
#         self.rosbag_button.setText(button_text)

#         if self._record_bags:
#             # Load the latest topic names from config file
#             ws_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

#             cfg_path = os.path.join(ws_path, "snakelib_gui", "param", "gui.yaml")
#             with open(cfg_path) as f:
#                 config = yaml.load(f)

#             time_now = datetime.now().strftime("%m-%d-%Y_%H-%M-%S")
#             save_dir = os.path.join(ws_path, "data", "rosbags")
#             save_path = os.path.join(save_dir, f"{time_now}.bag")

#             os.makedirs(save_dir, exist_ok=True)

#             self.bagger = RosbagRecorder(save_path, config["topics"])

#         elif hasattr(self, "bagger"):
#             # Stop recording
#             if self.bagger.is_recording:
#                 self.bagger.stop_recording_handler()

#     def joint_states_cb(self, msg):
#         self.joint_position = np.array(msg.position)
#         self.joint_velocity = np.array(msg.velocity)
#         self.joint_effort = np.array(msg.effort)

#         N = len(self.joint_velocity)

#         if N == 0:
#             return

#         self.node.declare_parameter("/snake_type", Parameter.Type.STRING)
#         snake_type = self.node.get_parameter("/snake_type")
#         # self.node.declare_parameter(snake_type+"/max_position", 1.6)
#         # self.node.declare_parameter(snake_type+"/max_velocity", 3.0)
#         # self.node.declare_parameter(snake_type+"/max_torque", 7.0)
#         self.node.declare_parameter(snake_type+"/max_position", Parameter.Type.DOUBLE)
#         self.node.declare_parameter(snake_type+"/max_velocity", Parameter.Type.DOUBLE)
#         self.node.declare_parameter(snake_type+"/max_torque", Parameter.Type.DOUBLE)
        
#         max_position = self.node.get_parameter(snake_type + "/max_position")
#         max_velocity = self.node.get_parameter(snake_type + "/max_velocity")
#         max_effort = self.node.get_parameter(snake_type + "/max_torque")

#         unsafe_modules = np.where(
#             np.logical_or(
#                 np.logical_or(self.joint_position > max_position, self.joint_position < -max_position),
#                 np.logical_or(self.joint_velocity > max_velocity, self.joint_velocity < -max_velocity),
#                 np.logical_or(self.joint_effort > max_effort, self.joint_effort < -max_effort),
#             )
#         )[0]

#         safe_modules = [i for i in range(N) if i not in unsafe_modules]

#         # Create joint positions plot
#         self.pos_figure.clear()
#         ax = self.pos_figure.add_subplot(111)
#         ax.bar(safe_modules, self.joint_position[safe_modules], color="g")
#         if len(unsafe_modules) != 0:
#             ax.bar(unsafe_modules, self.joint_position[unsafe_modules], color="r")
#         ax.set_title("Joint Positions")
#         ax.set_xlabel("Joint")
#         ax.set_ylabel("Position")
#         ax.set_ylim(-1.1 * max_position, 1.1 * max_position)
#         ax.set_xlim(0, N)
#         self.pos_canvas.draw()

#         # Create velocities plot
#         self.vel_figure.clear()
#         ax = self.vel_figure.add_subplot(111)
#         ax.bar(safe_modules, self.joint_velocity[safe_modules], color="g")
#         if len(unsafe_modules) != 0:
#             ax.bar(unsafe_modules, self.joint_velocity[unsafe_modules], color="r")
#         ax.set_title("Joint Velocities")
#         ax.set_xlabel("Joint")
#         ax.set_ylabel("Velocity")
#         ax.set_ylim(-1.1 * max_velocity, 1.1 * max_velocity)
#         ax.set_xlim(0, N)
#         self.vel_canvas.draw()

#         # Create effort plot
#         self.effort_figure.clear()
#         ax = self.effort_figure.add_subplot(111)
#         ax.bar(safe_modules, self.joint_effort[safe_modules], color="g")
#         if len(unsafe_modules) != 0:
#             ax.bar(unsafe_modules, self.joint_effort[unsafe_modules], color="r")
#         ax.set_title("Joint Efforts")
#         ax.set_xlabel("Joint")
#         ax.set_ylabel("Effort")
#         ax.set_ylim(-1.1 * max_effort, 1.1 * max_effort)
#         ax.set_xlim(0, N)
#         self.effort_canvas.draw()

#     def image_callback(self, msg):
#         pil_img = self.img_msg_to_pil(msg)
#         self.pixmap = QPixmap.fromImage(ImageQt(pil_img))
#         self.label.setPixmap(self.pixmap)

#     def ros_log_info_cb(self, msg):
#         self.log_info_msg = msg.msg
#         self.ui_robot_msgs.setText(f"Info: {self.log_info_msg}")

#     def robot_status_cb(self, msg):
#         self.robot_status = msg.data
#         self.ui_robot_status.setText(f"Robot mode: {msg.data}")

#     def set_display_enabled(self, display_names, value):
#         for name in display_names:
#             self.get_display(name).setEnabled(value == 2)

#     def switch_to_view(self, view_name):
#         view_man = self.manager.getViewManager()
#         for i in range(view_man.getNumViews()):
#             if view_man.getViewAt(i).getName() == view_name:
#                 view_man.setCurrentFrom(view_man.getViewAt(i))
#                 return
#         print("Did not find view named %s." % view_name)

#     def get_display(self, display_name):
#         displayGroup = self.manager.getRootDisplayGroup()
#         for i in range(displayGroup.numDisplays()):
#             if displayGroup.getDisplayAt(i).getName() == display_name:
#                 return displayGroup.getDisplayAt(i)


import os
import subprocess
import yaml
import numpy as np
from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QSizePolicy
)
from PyQt5.QtGui import QPixmap, QImage
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String
from rosgraph_msgs.msg import Log
from cv_bridge import CvBridge
import cv2

class SnakeViz(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # Layouts
        layout = QHBoxLayout()
        sub_layout = QVBoxLayout()

        # Camera feed label
        self.camera_label = QLabel("Camera feed")
        self.camera_label.setFixedSize(640, 480)
        sub_layout.addWidget(self.camera_label)

        # Matplotlib plots
        self.pos_figure = plt.figure(figsize=(4.0, 2.8))
        self.pos_canvas = FigureCanvas(self.pos_figure)
        sub_layout.addWidget(self.pos_canvas)

        self.vel_figure = plt.figure(figsize=(4.0, 2.8))
        self.vel_canvas = FigureCanvas(self.vel_figure)
        sub_layout.addWidget(self.vel_canvas)

        self.effort_figure = plt.figure(figsize=(4.0, 2.8))
        self.effort_canvas = FigureCanvas(self.effort_figure)
        sub_layout.addWidget(self.effort_canvas)

        # Buttons
        self.led_button = QPushButton("Toggle LED")
        self.led_button.clicked.connect(lambda: self.toggle_param("/led"))
        sub_layout.addWidget(self.led_button)

        self.overlay_img_button = QPushButton("Toggle thermal overlay")
        self.overlay_img_button.clicked.connect(lambda: self.toggle_param("/stream_overlay"))
        sub_layout.addWidget(self.overlay_img_button)

        self.rotate_img_button = QPushButton("Toggle roll compensation")
        self.rotate_img_button.clicked.connect(lambda: self.toggle_param("/rotate_image"))
        sub_layout.addWidget(self.rotate_img_button)

        self.rosbag_button = QPushButton("Start recording bags")
        self.rosbag_button.clicked.connect(self.ros_bags_record)
        sub_layout.addWidget(self.rosbag_button)

        # Status indicators
        self.ui_robot_status = QLabel("Robot mode indicator")
        sub_layout.addWidget(self.ui_robot_status)

        self.ui_robot_msgs = QLabel("ROS log info msgs")
        sub_layout.addWidget(self.ui_robot_msgs)

        layout.addLayout(sub_layout, 1)

        # Placeholder for 3D visualization info (can be replaced with pyqtgraph.opengl widget)
        self.visualization_label = QLabel("3D Visualization placeholder")
        self.visualization_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.visualization_label, 3)

        self.setLayout(layout)

        # ROS subscriptions
        self.joint_position = np.array([])
        self.joint_velocity = np.array([])
        self.joint_effort = np.array([])
        self.bridge = CvBridge()

        self.robot_status_sub = self.node.create_subscription(
            String,
            "/current_status",
            self.robot_status_cb,
            5)

        self.log_info_sub = self.node.create_subscription(
            Log,
            "/rosout_agg",
            self.ros_log_info_cb,
            5)

        self.joint_states_sub = self.node.create_subscription(
            JointState,
            "/snake/joint_states",
            self.joint_states_cb,
            1)

        self.image_sub = self.node.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            1
        )

        self._record_bags = False
        self.bagger = None

    def toggle_param(self, param_name):
        # Toggle boolean parameter on the ROS 2 node
        current = self.node.get_parameter(param_name).get_parameter_value().bool_value
        self.node.set_parameters([Parameter(param_name, Parameter.Type.BOOL, not current)])

    def ros_bags_record(self):
        # Simplified version of rosbag record start/stop
        self._record_bags = not self._record_bags
        if self._record_bags:
            self.rosbag_button.setText("Stop recording bags")
            # Launch rosbag record as a subprocess or implement your RosbagRecorder class
            print("Start rosbag recording - implement accordingly")
        else:
            self.rosbag_button.setText("Start recording bags")
            print("Stop rosbag recording - implement accordingly")

    def joint_states_cb(self, msg):
        self.joint_position = np.array(msg.position)
        self.joint_velocity = np.array(msg.velocity)
        self.joint_effort = np.array(msg.effort)
        self.update_joint_plots()

    def update_joint_plots(self):
        N = len(self.joint_velocity)
        if N == 0:
            return

        # Get max values from parameters
        snake_type = self.node.get_parameter("/snake_type").get_parameter_value().string_value
        max_position = self.node.get_parameter(snake_type + "/max_position").get_parameter_value().double_value
        max_velocity = self.node.get_parameter(snake_type + "/max_velocity").get_parameter_value().double_value
        max_effort = self.node.get_parameter(snake_type + "/max_torque").get_parameter_value().double_value

        unsafe_modules = np.where(
            np.logical_or(
                np.logical_or(self.joint_position > max_position, self.joint_position < -max_position),
                np.logical_or(self.joint_velocity > max_velocity, self.joint_velocity < -max_velocity),
                np.logical_or(self.joint_effort > max_effort, self.joint_effort < -max_effort),
            )
        )[0]

        safe_modules = [i for i in range(N) if i not in unsafe_modules]

        # Plot Joint Positions
        self.pos_figure.clear()
        ax = self.pos_figure.add_subplot(111)
        ax.bar(safe_modules, self.joint_position[safe_modules], color="g")
        if len(unsafe_modules) != 0:
            ax.bar(unsafe_modules, self.joint_position[unsafe_modules], color="r")
        ax.set_title("Joint Positions")
        ax.set_xlabel("Joint")
        ax.set_ylabel("Position")
        ax.set_ylim(-1.1 * max_position, 1.1 * max_position)
        ax.set_xlim(0, N)
        self.pos_canvas.draw()

        # Plot Velocities
        self.vel_figure.clear()
        ax = self.vel_figure.add_subplot(111)
        ax.bar(safe_modules, self.joint_velocity[safe_modules], color="g")
        if len(unsafe_modules) != 0:
            ax.bar(unsafe_modules, self.joint_velocity[unsafe_modules], color="r")
        ax.set_title("Joint Velocities")
        ax.set_xlabel("Joint")
        ax.set_ylabel("Velocity")
        ax.set_ylim(-1.1 * max_velocity, 1.1 * max_velocity)
        ax.set_xlim(0, N)
        self.vel_canvas.draw()

        # Plot Efforts
        self.effort_figure.clear()
        ax = self.effort_figure.add_subplot(111)
        ax.bar(safe_modules, self.joint_effort[safe_modules], color="g")
        if len(unsafe_modules) != 0:
            ax.bar(unsafe_modules, self.joint_effort[unsafe_modules], color="r")
        ax.set_title("Joint Efforts")
        ax.set_xlabel("Joint")
        ax.set_ylabel("Effort")
        ax.set_ylim(-1.1 * max_effort, 1.1 * max_effort)
        ax.set_xlim(0, N)
        self.effort_canvas.draw()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image, then to QPixmap for display
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        q_img = QPixmap.fromImage(
            QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        )
        self.camera_label.setPixmap(q_img)

    def ros_log_info_cb(self, msg):
        self.ui_robot_msgs.setText(f"Info: {msg.msg}")

    def robot_status_cb(self, msg):
        self.ui_robot_status.setText(f"Robot mode: {msg.data}")

    def launch_rviz(self, rviz_config_path=None):
        # Optional helper to launch RViz2 externally
        cmd = ["ros2", "run", "rviz2", "rviz2"]
        if rviz_config_path is not None:
            cmd.append("-d")
            cmd.append(rviz_config_path)
        subprocess.Popen(cmd)
