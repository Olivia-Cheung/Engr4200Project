import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import math

class OdometryNode(Node):
    
    def __init__(self):
        super().__init__('odometry')

        self.x = 0.0
        self.y = 0.0
        self.last_time = self.get_clock().now()

        self.heading = 0.0
        self.wheel_left = 0.0
        self.wheel_right = 0.0

        self.odometry_publisher = self.create_publisher(Odometry, 'auto_vehicle/odometry', 10)

        self.imu_subscription = self.create_subscription(Imu, 'auto_vehicle/imu', self.imu_callback, 10)

        self.speeds_subscription = self.create_subscription(Point, 'auto_vehicle/hardware/speeds', self.speeds_callback, 10)

        self.transform_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)

    def imu_callback(self, msg):
        orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        theta = tf_transformations.euler_from_quaternion(orientation_list)[2]

        self.heading = theta
    
    def speeds_callback(self, msg):
        self.wheel_left = msg.x - 7.0 * (5.00 / 0.8)
        self.wheel_right = (7.0 - msg.y) * (5.00 / 0.8)

    def timer_callback(self):
        current_time = self.get_clock().now()
        delta_time = current_time - self.last_time
        delta_time_sec = delta_time.nanoseconds / 1e9
        self.last_time = current_time

        v = (self.wheel_left + self.wheel_right) / 2.0

        self.x += v * delta_time_sec * math.sin(self.heading)
        self.y += v * delta_time_sec * math.cos(self.heading)

        self.get_logger().info("X: " + str(self.x) + " Y: " + str(self.y) + " Heading: " + str(math.degrees(self.heading)))
        
        quat = tf_transformations.quaternion_from_euler(0, 0, self.heading)

        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self.transform_broadcaster.sendTransform(transform)

        odometry_msg = Odometry()
        odometry_msg.header.stamp = current_time.to_msg()
        odometry_msg.pose.pose.position.x = self.x
        odometry_msg.pose.pose.position.y = self.y
        odometry_msg.pose.pose.position.z = 0.0
        odometry_msg.pose.pose.orientation.x = quat[0]
        odometry_msg.pose.pose.orientation.y = quat[1]
        odometry_msg.pose.pose.orientation.z = quat[2]
        odometry_msg.pose.pose.orientation.w = quat[3]
        self.odometry_publisher.publish(odometry_msg)

def main():
    rclpy.init()

    odometry = OdometryNode()

    rclpy.spin(odometry)

    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()