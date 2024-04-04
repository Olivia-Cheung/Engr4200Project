import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class Localization(Node):
    
    def __init__(self):
        super().__init__('localization')

def main():
    rclpy.init()

    localization = Localization()

    rclpy.spin(localization)

    localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()