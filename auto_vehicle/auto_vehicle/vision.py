import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import math

class Vision(Node):
    
    def __init__(self):
        super().__init__('vision')

        self.br = CvBridge()

        self.stop_data = cv2.CascadeClassifier('/home/vehicle/ros2_ws/src/auto_vehicle/models/stop_sign_classifier.xml')

        self.frame_publisher = self.create_publisher(CompressedImage, 'auto_vehicle/vision/frame', 10)
        self.stop_sign_publisher = self.create_publisher(Point, 'auto_vehicle/vision/stop_signs', 10)

        self.webcam_subscription = self.create_subscription(Image, 'auto_vehicle/webcam', self.webcam_callback, 10)

    def webcam_callback(self, msg):
        self.get_logger().info('Received webcam frame')

        frame = self.br.imgmsg_to_cv2(msg)

        # Stop Sign Detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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

        self.frame_publisher.publish(self.br.cv2_to_compressed_imgmsg(frame))

def main():
    rclpy.init()

    vision = Vision()

    rclpy.spin(vision)

    vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()