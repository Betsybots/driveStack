import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ackermann_drive_pkg_dir = get_package_share_directory('ackermann-drive')
    pkg_dir = get_package_share_directory('bringup')

    use_localization_arg = DeclareLaunchArgument(
        'use_localization',
        default_value='false',
        description='Launch the localization stack',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            ackermann_drive_pkg_dir,
            'config',
            'ackermannDrive.yaml',
        ),
        description='Path to ackermann-drive YAML config file',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'localization.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_localization')),
    )

    ackermann_drive_node = Node(
        package='ackermann-drive',
        executable='ackermann-drive',
        name='ackermann_drive_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        use_localization_arg,
        ackermann_drive_node,
        localization_launch,
    ])
