import rclpy
from rclpy.node import Node

class Vision(Node):
    
    def __init__(self):
        super().__init__('vision')

def main():
    rclpy.init()

    vision = Vision()

    rclpy.spin(vision)

    vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()