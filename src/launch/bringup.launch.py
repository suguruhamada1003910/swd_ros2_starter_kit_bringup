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

    bringup_dir = get_package_share_directory('bringup')
    swd_ros2_controllers_dir = get_package_share_directory(
        'swd_ros2_controllers')

    # Create the launch configuration variables
    baseline_m = LaunchConfiguration('baseline_m')

    # Nodes description
    def swd_diff_drive_controller_description():
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                swd_ros2_controllers_dir, 'launch', 'swd_diff_drive_controller.launch.py')]),
            launch_arguments={'baseline_m': baseline_m}.items()
        )

     # Launch description
    return LaunchDescription([
        # Declare the launch options
        DeclareLaunchArgument(
            'baseline_m',
            default_value='0.485',  # Starter-kit
        ),
        # SWD diff drive controller
        swd_diff_drive_controller_description(),
        # LiDAR 1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(bringup_dir, 'launch/lidar_idec_se2l.launch.py')]),
            launch_arguments={'prefix': 'laser'}.items()
        ),        
        # Xbox pad
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(bringup_dir, 'launch/xbox.launch.py')])
        )
    ])
