import sys, os
import os.path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # Load specific dbus user session if exists
    session = "/tmp/SYSTEMCTL_dbus.id"
    if os.path.isfile(session):
        print("SYSTEMCTL_dbus.id detected")

        with open(session) as f:
            lines = f.readlines()
        env = dict(line.strip().split("=", 1) for line in lines)
        os.environ.update(env)

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    baseline_m = LaunchConfiguration('baseline_m', default = '0.485')
    pub_freq_hz = LaunchConfiguration('pub_freq_hz', default = '50')
    base_frame = LaunchConfiguration('base_frame', default = 'base_footprint')
    odom_frame = LaunchConfiguration('odom', default = 'odom')
    motor_max_speed_rpm = LaunchConfiguration('motor_max_speed_rpm', default = '1050')

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'baseline_m',
            default_value='0.485'),
        DeclareLaunchArgument(
            'pub_freq_hz',
            default_value='50'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint'),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom'),
        DeclareLaunchArgument(
            'motor_max_speed_rpm',
            default_value='1050'),
        Node(
            package='swd_ros2_controllers',
            executable='swd_diff_drive_controller',
            name='swd_diff_drive_controller',
            parameters=[
                        {'use_sim_time': use_sim_time},
                        {"baseline_m": baseline_m},
                        {"left_swd_config_file": "/opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini"},
                        {"right_swd_config_file": "/opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini"},
                        {"pub_freq_hz": pub_freq_hz},
                        {"watchdog_receive_ms": 500},
                        {"base_frame": base_frame},
                        {"odom_frame": odom_frame},
                        {"publish_odom": True},
                        {"publish_tf": True},
                        {"publish_safety_functions": True},
                        {"motor_max_speed_rpm": motor_max_speed_rpm},
                        {"motor_max_safety_limited_speed_rpm": 490},
                        {"have_backward_sls": False},
                        {"left_encoder_relative_error": 0.2},
                        {"right_encoder_relative_error": 0.2},
                        ]
        )
    ])
