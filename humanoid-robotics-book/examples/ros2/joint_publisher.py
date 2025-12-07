import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class JointPublisher(Node):
    """
    A ROS 2 node that publishes joint commands to control a humanoid robot.
    This node publishes a sinusoidal motion to a "head_joint" and a "torso_joint".
    """
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Joint Publisher has been started.')

    def timer_callback(self):
        """
        Callback function for the timer.
        Creates and publishes a JointTrajectory message.
        """
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['head_joint', 'torso_joint']
        
        point = JointTrajectoryPoint()
        
        # Calculate sinusoidal position
        head_pos = 0.5 * math.sin(self.i * 0.1)
        torso_pos = 0.3 * math.sin(self.i * 0.05)
        
        point.positions = [head_pos, torso_pos]
        
        # Set a small duration for the point
        point.time_from_start = Duration(sec=0, nanosec=100000000) # 0.1 seconds
        
        msg.points.append(point)
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Publishing: head_joint={head_pos:.2f}, torso_joint={torso_pos:.2f}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        joint_publisher = JointPublisher()
        rclpy.spin(joint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'joint_publisher' in locals() and rclpy.ok():
            joint_publisher.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
