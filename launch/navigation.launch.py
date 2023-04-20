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
    bringup_dir = get_package_share_directory('swd_ros2_starter_kit_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            bringup_dir,
            'map',
            'map.yaml'))
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            bringup_dir,
            'params',
            'starter_kit_navigation_params.yaml'))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        bringup_dir,
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                bringup_dir, 'launch', 'swd_controller.launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
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
