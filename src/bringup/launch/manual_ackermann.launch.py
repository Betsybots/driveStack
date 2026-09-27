import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ackermann_drive_pkg_dir = get_package_share_directory('ackermann-drive')
    pkg_dir = get_package_share_directory('bringup')

    # Ackermann Drive
    # Joy_node, Teleop_twist_joy

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(ackermann_drive_pkg_dir, 'config', 'ackermannDrive.yaml'),
        description='Path to ackermann-drive YAML config file'
    )

    joy_config_file_arg = DeclareLaunchArgument(
        'joy_config_file',
        default_value=os.path.join(pkg_dir, 'config', 'ackermannJoystick.yaml'),
        description='Path to teleop_twist_joy YAML config file'
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
    )

    joy_teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            remappings=[('/cmd_vel', '/joy_cmd_vel')],
            parameters=[LaunchConfiguration('joy_config_file')]
    )

    # Joystick command bypasses the Nav2 velocity smoother, so it is sent straight
    # to the drive node -- same as the diff-drive manual launch did.
    ackermann_drive_node = Node(
            package='ackermann-drive',
            executable='ackermann-drive',
            name='ackermann_drive_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'cmd_vel_topic': '/joy_cmd_vel'},
            ]
    )

    return LaunchDescription([
        config_file_arg,
        joy_config_file_arg,
        joy_node,
        joy_teleop_node,
        ackermann_drive_node,
    ])
