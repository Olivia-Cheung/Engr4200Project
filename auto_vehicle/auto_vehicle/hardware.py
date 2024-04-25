import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Point

class Hardware(Node):
    
    safety_count = 0
    last_safety_count = 0

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

        self.timer = self.create_timer(1.0, self.timer_callback)

    def speeds_callback(self, point):
        left_speed = (7.0 + point.y) + point.x
        right_speed = (7.0 - point.y) + point.x

        self.get_logger().info('Setting wheel speeds: ' + str(left_speed) + ' ' + str(right_speed))

        if left_speed == 7.0 and right_speed == 7.0:
            self.left_wheel_pwm.ChangeDutyCycle(0)
            self.right_wheel_pwm.ChangeDutyCycle(0)
        else:
            self.left_wheel_pwm.ChangeDutyCycle(left_speed)
            self.right_wheel_pwm.ChangeDutyCycle(right_speed)

        self.safety_count += 1;

    def timer_callback(self):
        if self.last_safety_count >= self.safety_count:
            self.get_logger().info('Safety called')

            self.left_wheel_pwm.ChangeDutyCycle(0)
            self.right_wheel_pwm.ChangeDutyCycle(0)

        self.last_safety_count = self.safety_count
    
def main():
    rclpy.init()

    hardware = Hardware()

    rclpy.spin(hardware)

    hardware.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()