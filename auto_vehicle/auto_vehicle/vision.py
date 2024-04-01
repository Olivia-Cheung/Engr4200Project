import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class Vision(Node):
    
    def __init__(self):
        super().__init__('vision')

        self.br = CvBridge()

        self.frame_publisher = self.create_publisher(CompressedImage, 'auto_vehicle/vision/frame', 10)

        self.webcam_subscription = self.create_subscription(Image, 'auto_vehicle/webcam', self.webcam_callback, 10)

    def webcam_callback(self, msg):
        self.get_logger().info('Received webcam frame')

        frame = self.br.imgmsg_to_cv2(msg)

        # TODO Vision Code
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        self.frame_publisher.publish(self.br.cv2_to_compressed_imgmsg(gray))

def main():
    rclpy.init()

    vision = Vision()

    rclpy.spin(vision)

    vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()