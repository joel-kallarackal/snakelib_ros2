import rclpy
from rclpy.executors import ExternalShutdownException
from scripts.usb_camera_interface import FisheyeCameraStreamer


def main():
    rclpy.init()
    camera_interface = FisheyeCameraStreamer()
    try:
        camera_interface.start()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        camera_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()