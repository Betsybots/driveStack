import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')
    diff_drive_pkg_share = get_package_share_directory('differential-drive')
    default_plan = os.path.join(pkg_share, 'config', 'motion_plan.yaml')
    default_ekf_config = os.path.join(pkg_share, 'config', 'localization_ekf.yaml')
    default_diff_drive_config = os.path.join(
        diff_drive_pkg_share, 'config', 'diffDrive.yaml'
    )

    plan_file = LaunchConfiguration('plan_file')
    pose_topic = LaunchConfiguration('pose_topic')
    cmd_topic = LaunchConfiguration('cmd_topic')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_ekf': LaunchConfiguration('use_ekf'),
            'with_lidar': LaunchConfiguration('with_lidar'),
            'ekf_config_file': LaunchConfiguration('ekf_config_file'),
            'wheel_odom_topic': LaunchConfiguration('wheel_odom_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'lidar_odom_topic': LaunchConfiguration('lidar_odom_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'plan_file',
            default_value=default_plan,
            description='Absolute path to YAML motion plan file',
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/FinalOdometry',
            description='Pose input topic (geometry_msgs/msg/PoseStamped)',
        ),
        DeclareLaunchArgument(
            'cmd_topic',
            default_value='/motor_cmd_vel',
            description='Twist command output topic',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_diff_drive_config,
            description='Path to differential-drive YAML config file',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Fuse localization sensors with robot_localization EKF',
        ),
        DeclareLaunchArgument(
            'with_lidar',
            default_value='false',
            description='Include LiDAR odometry; automatically true when use_ekf is false',
        ),
        DeclareLaunchArgument(
            'ekf_config_file',
            default_value=default_ekf_config,
            description='Path to localization EKF YAML config file',
        ),
        DeclareLaunchArgument(
            'wheel_odom_topic',
            default_value='/wheel_odom',
            description='Wheel odometry topic',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='IMU topic',
        ),
        DeclareLaunchArgument(
            'lidar_odom_topic',
            default_value='/Odometry',
            description='LiDAR odometry topic',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/FinalOdometry',
            description='Localization odometry output topic',
        ),
        Node(
            package='plan_b',
            executable='motion_plan_executor_node',
            name='motion_plan_executor',
            output='screen',
            parameters=[{
                'plan_file': plan_file,
                'pose_topic': pose_topic,
                'cmd_topic': cmd_topic,
            }],
        ),
        Node(
            package='differential-drive',
            executable='differential-drive',
            name='differential_drive_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'cmd_vel_topic': cmd_topic},
            ],
        ),
        localization_launch,
    ])
