from email.policy import default
from math import remainder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # Create the launch configuration variables
    prefix = LaunchConfiguration('prefix')

    # Launch description
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'prefix',
            default_value='laser'
        ),
        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.04', '0', '0', '0', '0', '0', 'base_link', prefix] # Starter-kit type
        ),
        # LiDAR
        Node(
            package='urg_node',
            executable='urg_node_driver',
            namespace=prefix,
            parameters=[{'ip_address': '10.0.0.5', 'laser_frame_id': prefix}]
        )
    ])

