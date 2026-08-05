from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
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
    ])
    