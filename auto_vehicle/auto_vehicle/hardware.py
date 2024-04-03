import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

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

        self.get_logger().info('Setting test wheel speeds')
        self.left_wheel_pwm.ChangeDutyCycle(5)
        self.right_wheel_pwm.ChangeDutyCycle(5)

def main():
    rclpy.init()

    hardware = Hardware()

    rclpy.spin(hardware)

    hardware.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()