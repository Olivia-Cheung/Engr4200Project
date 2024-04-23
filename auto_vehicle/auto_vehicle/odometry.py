import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class Odometry(Node):
    
    def __init__(self):
        super().__init__('odometry')

def main():
    rclpy.init()

    odometry = Odometry()

    rclpy.spin(odometry)

    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()