import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class Planner(Node):
    
    stop_start_time = -1
    road = 0.0

    def __init__(self):
        super().__init__('planner')

        self.speeds_publisher = self.create_publisher(Point, 'auto_vehicle/hardware/speeds', 10)

        self.stop_sign_subscription = self.create_subscription(Point, 'auto_vehicle/localization/stop_signs', self.stop_sign_callback, 10)
        self.road_subscription = self.create_subscription(Point, 'auto_vehicle/vision/road', self.road_callback, 10)

        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)

    def stop_sign_callback(self, point):
        self.get_logger().info('Received stop sign point: ' + str(point.y))

        if point.y < 15.0 and point.y > 0.0 and self.stop_start_time == -1:
            self.stop_start_time = time.time_ns()

    def road_callback(self, point):
        self.get_logger().info('Received road point: ' + str(point.x))

        self.road = point.x

    def timer_callback(self):
        v = 0.9
        w = self.road * 0.001

        if self.stop_start_time != -1:
            if (time.time_ns() - self.stop_start_time) <= 3.0 * 1000000000:
                v = 0.0
                w = 0.0
            else:
                self.stop_start_time = -1

        self.get_logger().info('Speeds: ' + str(v) + ' ' + str(w))

        speeds_msg = Point()
        speeds_msg.x = w
        speeds_msg.y = v
        self.speeds_publisher.publish(speeds_msg)

def main():
    rclpy.init()

    planner = Planner()

    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()