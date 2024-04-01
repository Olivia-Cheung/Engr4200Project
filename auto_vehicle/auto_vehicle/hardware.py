import rclpy
from rclpy.node import Node

class Hardware(Node):
    
    def __init__(self):
        super().__init__('hardware')

def main():
    rclpy.init()

    hardware = Hardware()

    rclpy.spin(hardware)

    hardware.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()