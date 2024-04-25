import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

class Vision(Node):
    
    def __init__(self):
        super().__init__('vision')

        self.br = CvBridge()

        self.stop_data = cv2.CascadeClassifier('/home/vehicle/ros2_ws/src/auto_vehicle/models/stop_sign_classifier.xml')

        self.frame_publisher = self.create_publisher(CompressedImage, 'auto_vehicle/vision/frame', 10)
        self.stop_sign_publisher = self.create_publisher(Point, 'auto_vehicle/vision/stop_signs', 10)
        self.road_publisher = self.create_publisher(Point, 'auto_vehicle/vision/road', 10)

        self.webcam_subscription = self.create_subscription(Image, 'auto_vehicle/webcam', self.webcam_callback, 10)

    def webcam_callback(self, msg):
        self.get_logger().info('Received webcam frame')

        frame = self.br.imgmsg_to_cv2(msg)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Stop Sign Detection
        stop_signs = self.stop_data.detectMultiScale(gray, minNeighbors=3)

        if len(stop_signs) > 0:
            for (x, y, w, h) in stop_signs:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 5)

                distanceY = (4.5 * 512.0) / w

                targetAngle = ((x + (w / 2.0)) / 320) * 55.0
                scaledTargetAngle = -(55.0 / 2.0) + targetAngle

                distanceX = distanceY * math.sin(math.radians(scaledTargetAngle))

                sign_msg = Point()
                sign_msg.x = distanceX
                sign_msg.y = distanceY
                self.stop_sign_publisher.publish(sign_msg)

        # Road Line Detection
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        edges = cv2.Canny(blurred, 85, 85)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 10, minLineLength=10, maxLineGap=10)

        average = 0.0
        average_count = 0
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                slope = (y2 - y1) / (x2 - x1)

                if abs(slope) < 0.5:
                    continue

                average += 160 - x1
                average_count += 1

                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if average_count > 0:
                average /= average_count
        
        road = Point()
        road.x = average
        self.road_publisher.publish(road)

        self.get_logger().info('Road average: ' + str(average))

        cv2.circle(frame, (int(160 + average), 100), 0, (0, 0, 255), 20)

        self.frame_publisher.publish(self.br.cv2_to_compressed_imgmsg(frame))


def main():
    rclpy.init()

    vision = Vision()

    rclpy.spin(vision)

    vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()