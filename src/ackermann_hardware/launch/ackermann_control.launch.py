import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('ackermann_hardware')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', LaunchConfiguration('robot_description_file'),
                ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
            ]),
            value_type=str,
        )
    }

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
    )

    # Loads AckermannSystem from the <ros2_control> tags in the robot description.
    control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[robot_description, LaunchConfiguration('controllers_file')],
    )

    joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Takes geometry_msgs/Twist on /bicycle_steering_controller/reference_unstamped
    # and publishes /bicycle_steering_controller/odometry.
    steering_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['bicycle_steering_controller', '--controller-manager', '/controller_manager'],
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}],
            condition=IfCondition(LaunchConfiguration('use_joystick')),
    )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[LaunchConfiguration('joy_config_file')],
            remappings=[('/cmd_vel', '/bicycle_steering_controller/reference_unstamped')],
            condition=IfCondition(LaunchConfiguration('use_joystick')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Run without CAN or PWM; the plugin echoes commands back as states'),
        DeclareLaunchArgument(
            'use_joystick',
            default_value='false',
            description='Start joy_node and teleop_twist_joy driving the steering controller'),
        DeclareLaunchArgument(
            'robot_description_file',
            default_value=os.path.join(pkg_dir, 'urdf', 'ackermann_robot.urdf.xacro'),
            description='Robot xacro that includes ackermann.ros2_control.xacro'),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=os.path.join(pkg_dir, 'config', 'ackermann_controllers.yaml'),
            description='controller_manager and bicycle_steering_controller parameters'),
        DeclareLaunchArgument(
            'joy_config_file',
            default_value=os.path.join(pkg_dir, 'config', 'joystick.yaml'),
            description='teleop_twist_joy parameters for the Logitech F710'),
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        steering_controller_spawner,
        joy_node,
        teleop_node,
    ])
