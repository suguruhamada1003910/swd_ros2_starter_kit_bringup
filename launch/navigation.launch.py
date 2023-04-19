import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_context import LaunchContext
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.events.process.process_exited import ProcessExited

import time


def generate_launch_description():

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('swd_ros2_starter_kit_bringup'),
            'map',
            'map.yaml'))
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('swd_ros2_starter_kit_bringup'),
            'params',
            'starter_kit_navigation_params.yaml'))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('swd_ros2_starter_kit_bringup'),
        'rviz',
        'starter_kit_ros2.rviz')

    # Load specific dbus user session if exists
    session = "/tmp/SYSTEMCTL_dbus.id"
    if os.path.isfile(session):
        print("SYSTEMCTL_dbus.id detected")

        with open(session) as f:
            lines = f.readlines()
        env = dict(line.strip().split("=", 1) for line in lines)
        os.environ.update(env)

     # Launch description
    return LaunchDescription([
        # TF base_footprint to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ), 
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # SWD diff drive controller
        Node(
            package='swd_ros2_controllers',
            executable='swd_diff_drive_controller',
            name='swd_diff_drive_controller',
            parameters=[
                        {'use_sim_time': use_sim_time},
                        {"baseline_m": 0.485},
                        {"left_swd_config_file": "/opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini"},
                        {"right_swd_config_file": "/opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini"},
                        {"pub_freq_hz": 50},
                        {"watchdog_receive_ms": 500},
                        {"base_frame": "base_footprint"},
                        {"odom_frame": "odom"},
                        {"publish_odom": True},
                        {"publish_tf": True},
                        {"publish_safety_functions": True},
                        {"motor_max_speed_rpm": 1050},
                        {"motor_max_safety_limited_speed_rpm": 490},
                        {"have_backward_sls": False},
                        {"left_encoder_relative_error": 0.2},
                        {"right_encoder_relative_error": 0.2},
                        ]
        ),
        # TF base_link to laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.04', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
        # LiDAR
        Node(
            package='urg_node',
            executable='urg_node_driver',
            parameters=[{'ip_address': '10.0.0.5', 'laser_frame_id': 'laser'}]
        ),        
        # navigation
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        ),
        # rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])