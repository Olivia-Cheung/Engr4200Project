import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class Planner(Node):
    
    def __init__(self):
        super().__init__('planner')

        self.speeds_publisher = self.create_publisher(Point, 'auto_vehicle/hardware/speeds', 10)

        self.stop_sign_subscription = self.create_subscription(Point, 'auto_vehicle/vision/stop_signs', self.stop_sign_callback, 10)

        self.get_logger().info('Setting starting speed')
        speeds_msg = Point()
        speeds_msg.x = 7.8
        speeds_msg.y = 6.0
        self.speeds_publisher.publish(speeds_msg)

    def stop_sign_callback(self, point):
        self.get_logger().info('Received point: ' + str(point.y))
        speeds_msg = Point()

        if point.y <= 30.0:
            speeds_msg.x = 7.0
            speeds_msg.y = 7.0
        else:
            speeds_msg.x = 7.8
            speeds_msg.y = 6.0
        
        self.speeds_publisher.publish(speeds_msg)



def main():
    rclpy.init()

    planner = Planner()

    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()