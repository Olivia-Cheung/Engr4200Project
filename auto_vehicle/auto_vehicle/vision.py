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
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        masked = cv2.inRange(blurred, 50, 140)

        cropped = masked[100:240, :]

        contours, heirarchy = cv2.findContours(cropped, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        count = 0
        sumX = 0
        sumY = 0

        for c in contours:
            if cv2.contourArea(c) > 10000:
                count += 1
            
                M = cv2.moments(c)

                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                sumX += cX
                sumY += cY

        if count > 0:
            grandCX = int(float(sumX) / float(count))

            cv2.circle(frame, (grandCX, 150), 30, (0, 0, 255), -1)

            height, width = frame.shape[:2]

            error = (grandCX - (width / 2.0)) + 0.0

            self.get_logger().info('Road error: ' + str(error))

            road = Point()
            road.x = error
            self.road_publisher.publish(road)

        self.frame_publisher.publish(self.br.cv2_to_compressed_imgmsg(frame))


def main():
    rclpy.init()

    vision = Vision()

    rclpy.spin(vision)

    vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()