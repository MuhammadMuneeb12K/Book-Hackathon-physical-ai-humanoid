from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file to visualize the humanoid URDF model in RViz2.
    """
    # Path to the URDF file
    urdf_file_name = 'humanoid.urdf'
    urdf = os.path.join(
        get_package_share_directory('humanoid_robotics_book'), # Replace with your package name
        'urdf', # Assuming the urdf is in a 'urdf' folder in your package
        urdf_file_name)

    # Note: For this to work, you need to have a ROS 2 package and install the URDF file.
    # A simpler approach for local testing without a full package is to use a hardcoded path.
    # For now, we will assume the launch file is run from the 'examples/ros2' directory.
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, 'humanoid.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'robot_description': robot_desc}],
            arguments=[urdf_path]),

        # Joint State Publisher GUI
        # This allows you to move the joints of the robot manually.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(current_dir, 'default.rviz')]) # Assumes a default.rviz file exists
    ])

# To run this launch file:
# 1. Make sure you have a ROS 2 workspace and this file is part of a package.
# 2. Source your workspace: `source install/setup.bash`
# 3. Run the launch file: `ros2 launch <your_package_name> view_robot.launch.py`
#
# For simplicity, we are not creating a full package here.
# You would need a package.xml and setup.py to properly locate the urdf and rviz files.
# The user will need to have a ROS 2 environment set up to run this.
# Also, a simple default.rviz file is needed. I will create one.
