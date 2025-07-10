#!/usr/bin/env python

import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication

from snakelib_gui.gui import SignalWakeupHandler, SnakeViz
from snakelib_gui.gui_basic import GUIBasic

if __name__ == "__main__":
    rclpy.init()
    node = Node("SnakeLib_GUI")

    node.declare_parameter("/basic_gui", Parameter.Type.BOOL)
    basic_gui = node.get_parameter("/basic_gui")

    if not basic_gui:
        app = QApplication(sys.argv)

        snake_viz = SnakeViz(app, node)
        snake_viz.showMaximized()

        SignalWakeupHandler(snake_viz.frame)
        signal.signal(signal.SIGINT, lambda sig, _: app.quit())

        app.exec_()
        app.closeAllWindows()

    else:
        app = GUIBasic(node)
        signal.signal(signal.SIGINT, lambda sig, _: app.close())
