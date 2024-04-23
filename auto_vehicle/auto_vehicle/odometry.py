import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class Odometry(Node):
    
    def __init__(self):
        super().__init__('odometry')

        self.x = 0.0
        self.y = 0.0
        self.last_time = self.get_clock().now()

        self.odometry_publisher = self.create_publisher(Odometry, 'auto_vehicle/odometry', 10)

        self.imu_subscription = self.create_subscription(Imu, 'auto_vehicle/imu', self.imu_callback, 10)

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        delta_time = current_time - self.last_time
        delta_time_sec = delta_time.to_sec()
        self.last_time = current_time

        self.x += msg.linear_acceleration.x * delta_time_sec
        self.y += msg.linear_acceleration.y * delta_time_sec
        
        odometry_msg = Odometry()
        odometry_msg.header.stamp = current_time.to_msg()
        odometry_msg.pose.pose.position.x = self.x
        odometry_msg.pose.pose.position.y = self.y
        odometry_msg.pose.pose.posiiton.z = 0.0
        odometry_msg.pose.orientation = msg.orientation_covariance
        self.odometry_publisher.publish(odometry_msg)

def main():
    rclpy.init()

    odometry = Odometry()

    rclpy.spin(odometry)

    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()