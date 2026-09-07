import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    diff_drive_pkg_dir = get_package_share_directory('differential-drive')
    pkg_dir = get_package_share_directory('bringup')


    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(diff_drive_pkg_dir, 'config', 'diffDrive.yaml'),
        description='Path to differential-drive YAML config file'
    )

    ekf_config_file_arg = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=os.path.join(pkg_dir, 'config', 'ekf_custom.yaml'),
        description='Path to robot_localization EKF YAML config file'
    )

    diff_drive_node = Node(
            package='differential-drive',
            executable='differential-drive',
            name='differential_drive_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'cmd_vel_topic': '/cmd_vel'},
            ]
    )

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('ekf_config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        ekf_config_file_arg,
        diff_drive_node,
        ekf_node,
    ])
