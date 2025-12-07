import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    """
    A ROS 2 node that subscribes to an IMU sensor topic and logs the data.
    """
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('IMU Subscriber has been started.')

    def listener_callback(self, msg):
        """
        Callback function for the IMU subscription.
        Logs the received linear acceleration and angular velocity.
        """
        self.get_logger().info('--- New IMU Measurement ---')
        self.get_logger().info(f'Linear Acceleration: x={msg.linear_acceleration.x:.2f}, y={msg.linear_acceleration.y:.2f}, z={msg.linear_acceleration.z:.2f}')
        self.get_logger().info(f'Angular Velocity:    x={msg.angular_velocity.x:.2f}, y={msg.angular_velocity.y:.2f}, z={msg.angular_velocity.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    try:
        imu_subscriber = ImuSubscriber()
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if 'imu_subscriber' in locals() and rclpy.ok():
            imu_subscriber.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
