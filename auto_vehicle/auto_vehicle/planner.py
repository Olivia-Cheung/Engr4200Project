import rclpy
from rclpy.node import Node

class Planner(Node):
    
    def __init__(self):
        super().__init__('planner')

def main():
    rclpy.init()

    planner = Planner()

    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()