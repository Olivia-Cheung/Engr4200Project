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

    def canny(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 50, 150)
        return canny
    
    def region_of_interest(self, canny):
        height = canny.shape[0]
        width = canny.shape[1]
        mask = np.zeros_like(canny)
        triangle = np.array([
            [(0, height), (246, 234), (560, height)],
        ], np.int32)
        cv2.fillPoly(mask, triangle, 255)
        masked_image = cv2.bitwise_and(canny, mask)
        return masked_image
    
    def display_lines(self, img, lines):
        line_image = np.zeros_like(img)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
        return line_image
    
    def average_slope_intercept(self, image, lines):
        left_fit = []
        right_fit = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope, intercept = fit
                if slope < 0:  # left lane
                    left_fit.append((slope, intercept))
                else:  # right lane
                    right_fit.append((slope, intercept))
        if not left_fit or not right_fit:
            return None
        left_line = self.make_points(image, np.average(left_fit, axis=0))
        right_line = self.make_points(image, np.average(right_fit, axis=0))
        return [left_line, right_line]

    def make_points(self, image, line):
        slope, intercept = line
        y1 = int(image.shape[0])
        y2 = int(y1 * 7 / 10)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return [(x1, y1, x2, y2)]
    
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