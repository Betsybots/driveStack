import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("bringup"),
        'config',
        'twist_mux.yaml'
    )

    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('cmd_vel_out', '/cmd_vel')
            ]
        )
    ])

