import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

# Import the ImuSubscriber node from the examples directory
# This assumes that the 'examples' directory is in the PYTHONPATH or the test is run from the root.
# For a proper ROS 2 package, this would be handled by setup.py
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../examples/ros2')))
from imu_subscriber import ImuSubscriber

class TestImuSubscriber(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize ROS 2
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown ROS 2
        rclpy.shutdown()

    def setUp(self):
        # Create a new instance of the node for each test
        self.node = ImuSubscriber()

    def tearDown(self):
        # Destroy the node after each test
        self.node.destroy_node()

    def test_node_creation(self):
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'imu_subscriber')

    def test_subscription_exists(self):
        # Check if the subscription to the 'imu' topic exists
        # This is a bit indirect as rclpy does not expose a direct way to get active subscriptions
        # We can check if a subscriber has been created
        self.assertIsNotNone(self.node.subscription)
        self.assertEqual(self.node.subscription.topic_name, 'imu')

    def test_callback_receives_message(self):
        # This is an integration-style test for the callback
        # Create a dummy publisher to send a message
        publisher_node = rclpy.create_node('dummy_imu_publisher')
        publisher = publisher_node.create_publisher(Imu, 'imu', 10)

        # Create a mock for the logger to check if the callback is called
        original_logger_info = self.node.get_logger().info
        messages_received = []

        def mock_logger_info(message):
            messages_received.append(message)
            original_logger_info(message) # Call original for visibility during debug

        self.node.get_logger().info = mock_logger_info

        # Spin the subscriber node in a separate thread or use a simple spinner
        # For this simple test, we will create a one-shot executor
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.node)
        executor.add_node(publisher_node)

        # Publish a test message
        test_msg = Imu()
        test_msg.linear_acceleration.x = 1.0
        test_msg.angular_velocity.z = 0.5
        publisher.publish(test_msg)

        # Give some time for the message to be processed
        rclpy.spin_once(self.node, timeout_sec=1)
        rclpy.spin_once(publisher_node, timeout_sec=1)
        
        executor.spin_once(timeout_sec=1)

        # Check if the logger received the expected messages
        self.assertTrue(any("--- New IMU Measurement ---" in msg for msg in messages_received))
        self.assertTrue(any("Linear Acceleration: x=1.00" in msg for msg in messages_received))
        self.assertTrue(any("Angular Velocity:    x=0.00, y=0.00, z=0.00" in msg for msg in messages_received)) # Note: Angular velocity x,y should be 0.00 as per default msg

        # Clean up publisher node
        publisher_node.destroy_node()

if __name__ == '__main__':
    unittest.main()
