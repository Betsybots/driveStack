import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('differential-drive'),
        'config',
        'ekf_custom.yaml'
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            remappings=[('/cmd_vel', '/motor_cmd_vel')],
            parameters=[{
                'axis_linear.x': 4,
                'scale_linear.x': 0.75,
                'axis_angular.yaw': 3,
                'scale_angular.yaw': 0.75,
                'enable_button': 5,
            }]
        ),

        Node(
            package='differential-drive',
            executable='differential-drive',
            name='differential_drive_node'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
    ])
