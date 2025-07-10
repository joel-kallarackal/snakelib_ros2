import rclpy
from rclpy.executors import ExternalShutdownException
from scripts.image_processing import ImageProcessor

def main():
    rclpy.init()
    camera_interface = ImageProcessor()
    try:
        camera_interface.start()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        camera_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()