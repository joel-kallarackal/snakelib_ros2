import rclpy
from rclpy.executors import ExternalShutdownException
from scripts.thermal_camera_interface import ThermalCameraStreamer

def main():
    rclpy.init()
    camera_interface = ThermalCameraStreamer()
    try:
        camera_interface.start()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        camera_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()