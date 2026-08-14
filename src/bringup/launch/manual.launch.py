import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        default_value=os.path.join(pkg_dir, 'config', 'localization_ekf.yaml'),
        description='Path to localization EKF YAML config file'
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Fuse localization sensors with robot_localization EKF'
    )

    with_lidar_arg = DeclareLaunchArgument(
        'with_lidar',
        default_value='false',
        description='Include LiDAR odometry; automatically true when use_ekf is false'
    )

    wheel_odom_topic_arg = DeclareLaunchArgument(
        'wheel_odom_topic',
        default_value='/wheel_odom',
        description='Wheel odometry topic'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='IMU topic'
    )

    lidar_odom_topic_arg = DeclareLaunchArgument(
        'lidar_odom_topic',
        default_value='/Odometry',
        description='LiDAR odometry topic'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/FinalOdometry',
        description='Localization odometry output topic'
    )

    joy_config_file_arg = DeclareLaunchArgument(
        'joy_config_file',
        default_value=os.path.join(pkg_dir, 'config', 'joystick.yaml'),
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

    diff_drive_node = Node(
            package='differential-drive',
            executable='differential-drive',
            name='differential_drive_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'cmd_vel_topic': '/joy_cmd_vel'},
            ]
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_ekf': LaunchConfiguration('use_ekf'),
            'with_lidar': LaunchConfiguration('with_lidar'),
            'ekf_config_file': LaunchConfiguration('ekf_config_file'),
            'wheel_odom_topic': LaunchConfiguration('wheel_odom_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'lidar_odom_topic': LaunchConfiguration('lidar_odom_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
        }.items()
    )


    return LaunchDescription([
        config_file_arg,
        ekf_config_file_arg,
        use_ekf_arg,
        with_lidar_arg,
        wheel_odom_topic_arg,
        imu_topic_arg,
        lidar_odom_topic_arg,
        output_topic_arg,
        joy_config_file_arg,
        joy_node,
        joy_teleop_node,
        diff_drive_node,
        localization_launch,
    ])
