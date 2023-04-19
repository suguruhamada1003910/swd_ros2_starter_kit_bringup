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

    bringup_dir = get_package_share_directory('swd_ros2_starter_kit_bringup')
    slam_params = os.path.join(bringup_dir, "params", 'starter_kit_slam_params')

    rviz_config_dir = os.path.join(
        bringup_dir,
        'rviz',
        'starter_kit_ros2.rviz')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

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
        # SWD diff drive controller
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'baseline_m',
            default_value='0.485'),
        DeclareLaunchArgument(
            'motor_max_speed_rpm',
            default_value='1050'),
        Node(
            package='swd_ros2_controllers',
            executable='swd_diff_drive_controller',
            name='swd_diff_drive_controller',
            parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {"baseline_m": LaunchConfiguration('baseline_m')},
                        {"left_swd_config_file": "/opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini"},
                        {"right_swd_config_file": "/opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini"},
                        {"pub_freq_hz": 30},
                        {"watchdog_receive_ms": 500},
                        {"base_frame": "base_footprint"},
                        {"odom_frame": "odom"},
                        {"publish_odom": True},
                        {"publish_tf": True},
                        {"publish_safety_functions": True},
                        {"motor_max_speed_rpm": LaunchConfiguration('motor_max_speed_rpm')},
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
        # Xbox pad
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(bringup_dir, 'launch/xbox.launch.py')])
        ),
        # SLAM
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        Node(
            parameters=[
                {"slam_params_file": slam_params},
                {'use_sim_time': use_sim_time},
                ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox'
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
