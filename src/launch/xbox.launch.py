import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Launch description
    return LaunchDescription([
        # Nodes
        Node(
            package='joy_linux', 
            executable='joy_linux_node', 
            name='joy_linux_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'autorepeat_rate': 20.0,
            }]
        ),
        Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_twist_joy_node', 
            parameters=[{
                'axis_linear.x': 1, # Left thumb stick vertical
                'axis_angular.yaw': 3, # Left thumb stick horizontal
                'scale_linear.x': 1.0,
                'scale_linear_turbo.x': 1.3,
                'scale_angular.yaw': 0.7,
                'scale_angular_turbo.yaw': 1.4,
                'enable_button': 5, # R1 BUTTON
                'enable_turbo_button': 6, # L2 BUTTON/AXIS 
            }]
            # 0   L3 LEFT RIGHT AXIS    
            # 1   L1 FRONT BACK AXIS
            # 2   TRIANGLE BUTTON
            # 3   SQUARE BUTTON  
            # 4   L1 BUTTON     
            # 5   R1 BUTTON    
            # 6   L2 BUTTON/AXIS 
            # 7   R2 BUTTON/AXIS     
            # 8   SHARE BUTTON 
            # 9   OPTIONS BUTTON
        )
    ])
