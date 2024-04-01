import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class Webcam(Node):
    
    def __init__(self):
        super().__init__('webcam')

        self.img_publisher = self.create_publisher(Image, 'auto_vehicle/webcam', 10)
        self.compressed_img_publisher = self.create_publisher(CompressedImage, 'auto_vehicle/webcam/compressed', 10)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        self.br = CvBridge()

        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            self.img_publisher.publish(self.br.cv2_to_imgmsg(frame))
            self.compressed_img_publisher.publish(self.br.cv2_to_compressed_imgmsg(frame, 'jpeg'))

            self.get_logger().info('Published webcam frame')

def main():
    rclpy.init()

    webcam = Webcam()

    rclpy.spin(webcam)

    webcam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()