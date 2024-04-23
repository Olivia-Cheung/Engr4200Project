import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Point

class Hardware(Node):
    
    def __init__(self):
        super().__init__('hardware')

        left_wheel_pin = 33
        right_wheel_pin = 35

        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(left_wheel_pin, GPIO.OUT)
        GPIO.setup(right_wheel_pin, GPIO.OUT)

        self.left_wheel_pwm = GPIO.PWM(left_wheel_pin, 50)
        self.left_wheel_pwm.start(0)

        self.right_wheel_pwm = GPIO.PWM(right_wheel_pin, 50)
        self.right_wheel_pwm.start(0)

        self.speeds_subscription = self.create_subscription(Point, 'auto_vehicle/hardware/speeds', self.speeds_callback, 10)

    def speeds_callback(self, point):
        self.get_logger().info('Setting wheel speeds: ' + str(point.x) + ' ' + str(point.y))

        self.left_wheel_pwm.ChangeDutyCycle(point.x)
        self.right_wheel_pwm.ChangeDutyCycle(point.y)

def main():
    rclpy.init()

    hardware = Hardware()

    rclpy.spin(hardware)

    hardware.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()