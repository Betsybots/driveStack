import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    diff_drive_pkg_dir = get_package_share_directory('differential-drive')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(diff_drive_pkg_dir, 'config', 'diffDrive.yaml'),
        description='Path to differential-drive YAML config file'
    )
    
    diff_drive_node = Node(
            package='differential-drive',
            executable='differential-drive',
            name='differential_drive_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file')
            ]
    )

    return LaunchDescription([
        config_file_arg,
        diff_drive_node
    ])
