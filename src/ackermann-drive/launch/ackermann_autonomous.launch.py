import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ackermann-drive')
    default_params = os.path.join(pkg_dir, 'config', 'ackermannDrive.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the ackermann drive parameter file'),
        Node(
            package='ackermann-drive',
            executable='ackermann-drive',
            name='ackermann_drive',
            output='screen',
            parameters=[params_file],
        ),
    ])
