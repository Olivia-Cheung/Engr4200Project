import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import uuid
import time
import math

class Localization(Node):
    
    # Stop sign constants
    stop_sign_max_move_x = 100.0
    stop_sign_max_move_y = 7.0
    stop_sign_update_time = 1.0
    stop_sign_purge_time = 5.0

    def __init__(self):
        super().__init__('localization')

        self.active_stop_signs = []

        self.stop_sign_subscription = self.create_subscription(Point, 'auto_vehicle/vision/stop_signs', self.stop_sign_callback, 10)

        self.stop_sign_publisher = self.create_publisher(Point, 'auto_vehicle/localization/stop_signs', 10)

        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)

    def stop_sign_callback(self, point):
        best_match = None
        delta_best_x = None
        delta_best_y = None

        for sign in self.active_stop_signs:
            delta_x = sign['x'] - point.x
            delta_y = sign['y'] - point.y

            if abs(delta_x) < self.stop_sign_max_move_x and abs(delta_y) < self.stop_sign_max_move_y:
                if best_match is not None:
                    if math.sqrt((delta_x * delta_x) + (delta_y * delta_y)) < math.sqrt((delta_best_x * delta_best_x) + (delta_best_y * delta_best_y)):
                        best_match = sign
                        delta_best_x = delta_x
                        delta_best_y = delta_y
                else:
                    best_match = sign
                    delta_best_x = delta_x
                    delta_best_y = delta_y
        
        if best_match is not None:
            best_match['x'] = point.x
            best_match['y'] = point.y
            best_match['last_updated'] = time.time_ns()
        else:
            temp_sign = {
                'id': uuid.uuid4(),
                'x': point.x,
                'y': point.y,
                'last_updated': time.time_ns()
            }

            self.active_stop_signs.append(temp_sign)
    
    def timer_callback(self):
        # Purge expired stop signs
        for sign in self.active_stop_signs:
            self.get_logger().info(str(len(self.active_stop_signs)) + ': ' + str(sign))

            if time.time_ns() - sign['last_updated'] >= self.stop_sign_update_time * 1000000000:
                sign['y'] -= 20.0 / 60.0
            
            sign_msg = Point()
            sign_msg.x = sign['x']
            sign_msg.y = sign['y']
            self.stop_sign_publisher.publish(sign_msg)

            if time.time_ns() - sign['last_updated'] >= self.stop_sign_purge_time * 1000000000:
                self.active_stop_signs.remove(sign)

def main():
    rclpy.init()

    localization = Localization()

    rclpy.spin(localization)

    localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()